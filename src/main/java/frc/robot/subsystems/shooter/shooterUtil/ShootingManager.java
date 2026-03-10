package frc.robot.subsystems.shooter.shooterUtil;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shooter.ShooterConstants.Mechanical.flywheelRadius;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants.Mechanical;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingManager {
  public static final Transform3d[] ROBOT_TO_PHOTON_CAMS =
      new Transform3d[] {
        new Transform3d(
            -Units.inchesToMeters(12.0),
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(6.0),
            new Rotation3d(0.0, -Units.degreesToRadians(150.0), Units.degreesToRadians(0.0))),
        new Transform3d(
            -Units.inchesToMeters(12.0),
            -Units.inchesToMeters(12.0),
            Units.inchesToMeters(6.0),
            new Rotation3d(0.0, -Units.degreesToRadians(150.0), Units.degreesToRadians(0.0)))
      };

  private static final InterpolatingTreeMap<Double, ShotParams> distanceToShotParams =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (startValue, endValue, t) ->
              new ShotParams(
                  Interpolator.forDouble()
                      .interpolate(startValue.flywheelRpm, endValue.flywheelRpm, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.hoodAngleRad, endValue.hoodAngleRad, t)));

  // TODO: Replace with new coefficients for RPM = a + b*x + c*x^2 + d*x^3 where x is distance
  // meters
  private static final double[] LOOKUP_TABLE_POLYNOMIAL =
      new double[] {3.34225381e+03, -3.57277105e-11, 1.40317141e-11, -1.76224478e-12};
  // TODO: Replace with new coefficients for hood angle deg = a + b*x + c*x^2 + d*x^3 where x is
  // distance meters
  private static final double[] LOOKUP_TABLE_HOOD_POLYNOMIAL =
      new double[] {108.40639278, -29.73013306, 8.33220425, -1.08310321};
  private static final double POLY_RPM_WEIGHT =
      0.5; // 0 = use map only, 1 = use poly only, 0.5 = blend both
  private static final double POLY_HOOD_WEIGHT =
      0.5; // 0 = use map only, 1 = use poly only, 0.5 = blend both

  public static final double MAX_ACCEL = 8.0;
  public static final double MAX_VELOCITY = 8.0;

  private static final double GRAVITY_MPS2 = 9.80665;
  private static final double MAX_FLYWHEEL_RPM = 6271.0;
  private static final double RPM_RATE_LIMIT = 600.0;
  private static final double LEAD_PHASE_DELAY_SEC = 0.03;
  private static final int LEAD_LOOKAHEAD_ITERATIONS = 20;

  private static final double RPM_TOLERANCE = 50.0;
  private static final double STABLE_RPM_TIME_SEC = 0.15;
  private static final double RECOVERY_TIME_SEC = 0.25;

  private double lastShotTimestamp = -Double.MAX_VALUE;
  private double rpmStableSince = -Double.MAX_VALUE;
  private double lastCommandedRpm = 0.0;
  private double lastCommandTimestamp = -Double.MAX_VALUE;

  static {
    // TODO: Replace with calibrated distance->shot params (meters, RPM, hood angle deg)
    addShotParams(1.581, 3342.253805, 78.0);
    addShotParams(1.94, 3342.253805, 74.0);
    addShotParams(2.414, 3342.253805, 72.0);
    addShotParams(2.43, 3342.253805, 68.0);
    addShotParams(2.75, 3342.253805, 67.0);
    addShotParams(3.11, 3342.253805, 64.0);
  }

  public ShootingManager() {
    this(null, null, null);
  }

  public ShootingManager(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      Supplier<Rotation2d> headingSupplier) {}

  public ShotParams getStaticShootingParams(double distanceMeters) {
    ShotParams params = distanceToShotParams.get(distanceMeters);
    double rpm = 0.0;
    double mapRpm = params != null ? params.flywheelRpm : Double.NaN;
    double polyRpm = evaluatePolynomial(LOOKUP_TABLE_POLYNOMIAL, distanceMeters);
    boolean mapValid = Double.isFinite(mapRpm) && mapRpm > 0.0;
    boolean polyValid = Double.isFinite(polyRpm) && polyRpm > 0.0;
    double hoodAngleRad;
    double minHoodRad = Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg);
    double maxHoodRad = Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg);
    double mapHoodRad = params != null ? params.hoodAngleRad : Double.NaN;
    double polyHoodDeg = evaluatePolynomial(LOOKUP_TABLE_HOOD_POLYNOMIAL, distanceMeters);
    double polyHoodRad = Units.degreesToRadians(polyHoodDeg);
    boolean mapHoodValid = Double.isFinite(mapHoodRad);
    boolean polyHoodValid = Double.isFinite(polyHoodRad);
    if (mapHoodValid && polyHoodValid) {
      hoodAngleRad = blend(mapHoodRad, polyHoodRad, POLY_HOOD_WEIGHT);
    } else if (mapHoodValid) {
      hoodAngleRad = mapHoodRad;
    } else if (polyHoodValid) {
      hoodAngleRad = polyHoodRad;
    } else {
      double fallbackDeg =
          (HoodConstants.Targeting.minAngleDeg + HoodConstants.Targeting.maxAngleDeg) * 0.5;
      hoodAngleRad = Units.degreesToRadians(fallbackDeg);
    }
    hoodAngleRad = MathUtil.clamp(hoodAngleRad, minHoodRad, maxHoodRad);

    if (mapValid && polyValid) {
      rpm = blend(mapRpm, polyRpm, POLY_RPM_WEIGHT);
    } else if (mapValid) {
      rpm = mapRpm;
    } else if (polyValid) {
      rpm = polyRpm;
    }
    return new ShotParams(MathUtil.clamp(rpm, 0.0, MAX_FLYWHEEL_RPM), hoodAngleRad);
  }

  private static void addShotParams(double distanceMeters, double rpm, double hoodAngleDeg) {
    distanceToShotParams.put(
        distanceMeters, new ShotParams(rpm, Units.degreesToRadians(hoodAngleDeg)));
  }

  private static double evaluatePolynomial(double[] coeffs, double x) {
    if (coeffs == null || coeffs.length == 0 || !Double.isFinite(x)) {
      return Double.NaN;
    }
    // If all coefficients are (effectively) zero the polynomial is uninitialized.
    boolean anyNonZero = false;
    for (double c : coeffs) {
      if (Math.abs(c) > 1e-12) {
        anyNonZero = true;
        break;
      }
    }
    if (!anyNonZero) {
      return Double.NaN;
    }

    double result = 0.0;
    double power = 1.0;
    for (double coeff : coeffs) {
      result += coeff * power;
      power *= x;
    }
    return result;
  }

  private static double blend(double a, double b, double weightB) {
    double clampedWeight = MathUtil.clamp(weightB, 0.0, 1.0);
    return a + (b - a) * clampedWeight;
  }

  public ShotSolution calculateShotSolution(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation3d fixedTarget,
      double headingToleranceMeters,
      double pitchToleranceMeters) {
  Pose2d estimatedRobotPose =
    robotPose.exp(
      new edu.wpi.first.math.geometry.Twist2d(
        robotRelativeSpeeds.vxMetersPerSecond * LEAD_PHASE_DELAY_SEC,
        robotRelativeSpeeds.vyMetersPerSecond * LEAD_PHASE_DELAY_SEC,
        robotRelativeSpeeds.omegaRadiansPerSecond * LEAD_PHASE_DELAY_SEC));

  Pose3d shooterPose = new Pose3d(estimatedRobotPose).plus(Mechanical.shooterPose);
  Translation3d shooterToTarget = fixedTarget.minus(shooterPose.getTranslation());

  double distanceMeters = shooterToTarget.toTranslation2d().getNorm();
  ShotParams staticParams = getStaticShootingParams(distanceMeters);
  double staticExitVelocity =
    calculateExitVelocityMetersPerSecondFromRpm(staticParams.flywheelRpm);

  ChassisSpeeds fieldSpeeds =
    ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, estimatedRobotPose.getRotation());
  Translation3d fieldVelocity =
    new Translation3d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, 0.0);

  Translation3d lookaheadShooterPosition = shooterPose.getTranslation();
  Translation3d lookaheadVector = shooterToTarget;
  double lookaheadDistanceMeters = distanceMeters;
  double timeOfFlightSec = Math.max(0.0, distanceMeters / Math.max(1e-6, staticExitVelocity));

  for (int i = 0; i < LEAD_LOOKAHEAD_ITERATIONS; i++) {
    ShotParams iterParams = getStaticShootingParams(lookaheadDistanceMeters);
    double iterExitVelocity = calculateExitVelocityMetersPerSecondFromRpm(iterParams.flywheelRpm);
    timeOfFlightSec = lookaheadDistanceMeters / Math.max(1e-6, iterExitVelocity);

    Translation3d offset = fieldVelocity.times(timeOfFlightSec);
    lookaheadShooterPosition = shooterPose.getTranslation().plus(offset);
    lookaheadVector = fixedTarget.minus(lookaheadShooterPosition);
    lookaheadDistanceMeters = lookaheadVector.toTranslation2d().getNorm();
  }

  ShotParams ledStaticParams = getStaticShootingParams(lookaheadDistanceMeters);
  double ledExitVelocity = calculateExitVelocityMetersPerSecondFromRpm(ledStaticParams.flywheelRpm);

  double yaw = Math.atan2(shooterToTarget.getY(), shooterToTarget.getX());
  double finalYaw = Math.atan2(lookaheadVector.getY(), lookaheadVector.getX());
  double pitchStatic = ledStaticParams.hoodAngleRad;
    double minPitch = Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg);
    double maxPitch = Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg);
    double clampedPitchStatic = MathUtil.clamp(pitchStatic, minPitch, maxPitch);

    Translation3d vStatic =
        new Translation3d(
      ledExitVelocity * Math.cos(clampedPitchStatic) * Math.cos(finalYaw),
      ledExitVelocity * Math.cos(clampedPitchStatic) * Math.sin(finalYaw),
      ledExitVelocity * Math.sin(clampedPitchStatic));

  Translation3d vRobot = fieldVelocity;

    Translation3d vFinal = vStatic.minus(vRobot);
    double finalPitch = Math.atan2(vFinal.getZ(), vFinal.toTranslation2d().getNorm());
    double clampedFinalPitch = MathUtil.clamp(finalPitch, minPitch, maxPitch);
    double finalExitVelocity = vFinal.getNorm();
    double rawRpm = calculateFlywheelRpmFromExitVelocity(finalExitVelocity);
    double limitedRpm = limitRpm(rawRpm, Timer.getFPGATimestamp());

  double yawErrorMeters =
    lookaheadDistanceMeters * Math.abs(MathUtil.angleModulus(finalYaw - yaw));
  double pitchErrorMeters =
    lookaheadDistanceMeters * Math.abs(clampedFinalPitch - clampedPitchStatic);

    Logger.recordOutput("ShootingManager/LeadYawDeg", Units.radiansToDegrees(finalYaw - yaw));
    Logger.recordOutput("ShootingManager/FinalPitchDeg", Units.radiansToDegrees(clampedFinalPitch));
    Logger.recordOutput(
        "ShootingManager/StaticPitchDeg", Units.radiansToDegrees(clampedPitchStatic));
  Logger.recordOutput("ShootingManager/LookaheadDistanceMeters", lookaheadDistanceMeters);
  Logger.recordOutput("ShootingManager/TimeOfFlightSec", timeOfFlightSec);
    Logger.recordOutput("ShootingManager/RawRadPerSec", rawRpm * (2 * Math.PI / 60));
    Logger.recordOutput("ShootingManager/LimitedRadPerSec", limitedRpm * (2 * Math.PI / 60));
    Logger.recordOutput("ShootingManager/HoodPitchDeg", Units.radiansToDegrees(finalPitch));

    return new ShotSolution(
        Rotation2d.fromRadians(finalYaw),
        Units.radiansToDegrees(clampedFinalPitch),
        limitedRpm,
    lookaheadDistanceMeters,
        yawErrorMeters <= headingToleranceMeters,
        pitchErrorMeters <= pitchToleranceMeters);
  }

  private double limitRpm(double targetRpm, double timestampSec) {
    double clamped = MathUtil.clamp(targetRpm, 0.0, MAX_FLYWHEEL_RPM);
    if (!Double.isFinite(lastCommandTimestamp)) {
      lastCommandTimestamp = timestampSec;
      lastCommandedRpm = clamped;
      return clamped;
    }
    double dt = timestampSec - lastCommandTimestamp;
    if (dt <= 0.0 || dt > 1.0) {
      lastCommandTimestamp = timestampSec;
      lastCommandedRpm = clamped;
      return clamped;
    }
    double maxDelta = RPM_RATE_LIMIT * dt;
    double limited =
        lastCommandedRpm + MathUtil.clamp(clamped - lastCommandedRpm, -maxDelta, maxDelta);
    lastCommandedRpm = limited;
    lastCommandTimestamp = timestampSec;
    return limited;
  }

  public Translation3d[] calculateTrajectory(
      Pose2d robotPose,
      ShotSolution solution,
      Translation3d fixedTarget,
      double timeStepSec,
      double maxTimeSec) {
    Pose3d shooterPose = new Pose3d(robotPose).plus(Mechanical.shooterPose);
    Translation3d origin = shooterPose.getTranslation();

    double exitVelocity = calculateExitVelocityMetersPerSecondFromRpm(solution.flywheelRpm);
    double yaw = solution.drivetrainHeading.getRadians();
    double pitch = solution.hoodPitchRad;

    Translation3d velocity =
        new Translation3d(
            exitVelocity * Math.cos(pitch) * Math.cos(yaw),
            exitVelocity * Math.cos(pitch) * Math.sin(yaw),
            exitVelocity * Math.sin(pitch));

    List<Translation3d> points = new ArrayList<>();
    double targetZ = fixedTarget.getZ();

    for (double t = 0.0; t <= maxTimeSec; t += timeStepSec) {
      double x = origin.getX() + velocity.getX() * t;
      double y = origin.getY() + velocity.getY() * t;
      double z = origin.getZ() + velocity.getZ() * t - 0.5 * GRAVITY_MPS2 * t * t;
      points.add(new Translation3d(x, y, z));
      if (t > 0.05 && z <= targetZ) {
        break;
      }
    }

    return points.toArray(new Translation3d[0]);
  }

  public boolean isFlywheelStable(double currentRpm, double targetRpm, double timestampSec) {
    if (Math.abs(currentRpm - targetRpm) <= RPM_TOLERANCE) {
      if (timestampSec - rpmStableSince >= STABLE_RPM_TIME_SEC) {
        return true;
      }
      if (rpmStableSince < 0.0 || timestampSec - rpmStableSince > 1.0) {
        rpmStableSince = timestampSec;
      }
    } else {
      rpmStableSince = -Double.MAX_VALUE;
    }
    return false;
  }

  public void recordShot(double timestampSec) {
    lastShotTimestamp = timestampSec;
  }

  public boolean canFire(ShotSolution solution, boolean rpmStable, double timestampSec) {
    boolean withinAngle = solution.yawWithinTolerance && solution.pitchWithinTolerance;
    boolean recovered = timestampSec - lastShotTimestamp >= RECOVERY_TIME_SEC;
    return withinAngle && rpmStable && recovered;
  }

  public double getCurrentTime() {
    return Timer.getFPGATimestamp();
  }

  public static class ShotParams {
    public final double flywheelRpm;
    public final double hoodAngleRad;

    public ShotParams(double flywheelRpm, double hoodAngleRad) {
      this.flywheelRpm = flywheelRpm;
      this.hoodAngleRad = hoodAngleRad;
    }
  }

  public static class ShotSolution {
    public final Rotation2d drivetrainHeading;
    public final double hoodPitchRad;
    public final double flywheelRpm;
    public final double distanceMeters;
    public final boolean yawWithinTolerance;
    public final boolean pitchWithinTolerance;

    public ShotSolution(
        Rotation2d drivetrainHeading,
        double hoodPitchRad,
        double flywheelRpm,
        double distanceMeters,
        boolean yawWithinTolerance,
        boolean pitchWithinTolerance) {
      this.drivetrainHeading = drivetrainHeading;
      this.hoodPitchRad = hoodPitchRad;
      this.flywheelRpm = flywheelRpm;
      this.distanceMeters = distanceMeters;
      this.yawWithinTolerance = yawWithinTolerance;
      this.pitchWithinTolerance = pitchWithinTolerance;
    }
  }

  private double calculateFlywheelRpmFromExitVelocity(double exitVelocityMps) {
    if (exitVelocityMps <= 0.0) {
      return 0.0;
    }
    double omegaRadPerSec = exitVelocityMps / flywheelRadius.in(Meters);
    return Units.radiansPerSecondToRotationsPerMinute(omegaRadPerSec);
  }

  private double calculateExitVelocityMetersPerSecondFromRpm(double flywheelRpm) {
    double omegaRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheelRpm);
    return omegaRadPerSec * flywheelRadius.in(Meters);
  }
}

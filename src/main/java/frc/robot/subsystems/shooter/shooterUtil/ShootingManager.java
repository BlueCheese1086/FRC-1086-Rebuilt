package frc.robot.subsystems.shooter.shooterUtil;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shooter.ShooterConstants.Mechanical.flywheelRadius;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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
      new double[] {6123.98722364, -3372.91648691, 1172.73628787, -116.49524005};
  // TODO: Replace with new coefficients for hood angle deg = a + b*x + c*x^2 + d*x^3 where x is
  // distance meters
  private static final double[] LOOKUP_TABLE_HOOD_POLYNOMIAL =
      new double[] {157.65295921, -87.06823815, 26.34973988, -2.55605388};
  private static final double POLY_RPM_WEIGHT =
      0.5; // 0 = use map only, 1 = use poly only, 0.5 = blend both
  private static final double POLY_HOOD_WEIGHT =
      0.5; // 0 = use map only, 1 = use poly only, 0.5 = blend both

  public static final double MAX_ACCEL = 8.0;
  public static final double MAX_VELOCITY = 8.0;

  private static final double GRAVITY_MPS2 = 9.80665;
  private static final double MAX_FLYWHEEL_RPM = 6271.0;
  private static final double RPM_RATE_LIMIT = 600.0;

  private static final double RPM_TOLERANCE = 50.0;
  private static final double STABLE_RPM_TIME_SEC = 0.15;
  private static final double RECOVERY_TIME_SEC = 0.25;

  private double lastShotTimestamp = -Double.MAX_VALUE;
  private double rpmStableSince = -Double.MAX_VALUE;
  private double lastCommandedRpm = 0.0;
  private double lastCommandTimestamp = -Double.MAX_VALUE;
  private double lastHeadingRad = Double.NaN;
  private double lastHeadingTimestamp = -Double.MAX_VALUE;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final Supplier<Rotation2d> headingSupplier;

  static {
    // TODO: Replace with calibrated distance->shot params (meters, RPM, hood angle deg)
    addShotParams(1.4478, 3351.803102, 78.0);
    addShotParams(1.8, 3151.267873, 65.0);
    addShotParams(2.054, 3151.267873, 75.0);
    addShotParams(3.048, 3437.746771, 65.0);
    addShotParams(5.334, 3819.718634, 55.0);
  }

  public ShootingManager() {
    this(null, null, null);
  }

  public ShootingManager(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      Supplier<Rotation2d> headingSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.headingSupplier = headingSupplier;
  }
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

  private double estimateAngularVelocity(Rotation2d heading, double timestampSec) {
    if (heading == null || !Double.isFinite(timestampSec)) {
      return 0.0;
    }
    if (!Double.isFinite(lastHeadingTimestamp) || timestampSec <= lastHeadingTimestamp) {
      lastHeadingRad = heading.getRadians();
      lastHeadingTimestamp = timestampSec;
      return 0.0;
    }
    double dt = timestampSec - lastHeadingTimestamp;
    if (dt <= 1e-6) {
      return 0.0;
    }
    double delta = MathUtil.angleModulus(heading.getRadians() - lastHeadingRad);
    lastHeadingRad = heading.getRadians();
    lastHeadingTimestamp = timestampSec;
    return delta / dt;
  }

  public ShotSolution calculateShotSolution(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation3d fixedTarget,
      double headingToleranceMeters,
      double pitchToleranceMeters) {
    Pose3d shooterPose = new Pose3d(robotPose).plus(Mechanical.shooterPose);
    Translation3d shooterToTarget = fixedTarget.minus(shooterPose.getTranslation());

    double distanceMeters = shooterToTarget.toTranslation2d().getNorm();
    ShotParams staticParams = getStaticShootingParams(distanceMeters);
    double staticExitVelocity =
        calculateExitVelocityMetersPerSecondFromRpm(staticParams.flywheelRpm);

    double yaw = Math.atan2(shooterToTarget.getY(), shooterToTarget.getX());
    double pitchStatic = staticParams.hoodAngleRad;
    double minPitch = Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg);
    double maxPitch = Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg);
    double clampedPitchStatic = MathUtil.clamp(pitchStatic, minPitch, maxPitch);

    Translation3d vStatic =
        new Translation3d(
            staticExitVelocity * Math.cos(clampedPitchStatic) * Math.cos(yaw),
            staticExitVelocity * Math.cos(clampedPitchStatic) * Math.sin(yaw),
            staticExitVelocity * Math.sin(clampedPitchStatic));

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());
    Translation3d vRobot =
        new Translation3d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, 0.0);

    Translation3d vFinal = vStatic.minus(vRobot);
    double finalYaw = Math.atan2(vFinal.getY(), vFinal.getX());
    double finalPitch = Math.atan2(vFinal.getZ(), vFinal.toTranslation2d().getNorm());
    double clampedFinalPitch = MathUtil.clamp(finalPitch, minPitch, maxPitch);
    double finalExitVelocity = vFinal.getNorm();
    double rawRpm = calculateFlywheelRpmFromExitVelocity(finalExitVelocity);
    double limitedRpm = limitRpm(rawRpm, Timer.getFPGATimestamp());

    double yawErrorMeters = distanceMeters * Math.abs(MathUtil.angleModulus(finalYaw - yaw));
    double pitchErrorMeters = distanceMeters * Math.abs(clampedFinalPitch - clampedPitchStatic);

    Logger.recordOutput("ShootingManager/LeadYawDeg", Units.radiansToDegrees(finalYaw - yaw));
    Logger.recordOutput("ShootingManager/FinalPitchDeg", Units.radiansToDegrees(clampedFinalPitch));
    Logger.recordOutput(
        "ShootingManager/StaticPitchDeg", Units.radiansToDegrees(clampedPitchStatic));
    Logger.recordOutput("ShootingManager/RawRpm", rawRpm * (2 * Math.PI / 60));
    Logger.recordOutput("ShootingManager/LimitedRpm", limitedRpm * (2 * Math.PI / 60));
    Logger.recordOutput("ShootingManager/HoodPitchDeg", Units.radiansToDegrees(finalPitch));

    return new ShotSolution(
        Rotation2d.fromRadians(finalYaw),
        clampedFinalPitch,
        limitedRpm,
        distanceMeters,
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

  public boolean canFire(
      ShotSolution solution, boolean rpmStable, double timestampSec) {
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

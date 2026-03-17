package frc.robot.subsystems.shooter.shooterUtil;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shooter.ShooterConstants.Mechanical.flywheelRadius;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  private static final InterpolatingTreeMap<Double, ShotParams> distanceToShotParams =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (startValue, endValue, t) ->
              new ShotParams(
                  Interpolator.forDouble()
                      .interpolate(startValue.flywheelRpm, endValue.flywheelRpm, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.hoodAngleRad, endValue.hoodAngleRad, t)));
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // TODO: Replace with new coefficients for RPM = a + b*x + c*x^2 + d*x^3 where x is distance
  // meters
  private static final double[] LOOKUP_TABLE_POLYNOMIAL =
      new double[] {1581.2855209, 1995.45539188, -661.82335317, 73.73303471};
  // TODO: Replace with new coefficients for hood angle deg = a + b*x + c*x^2 + d*x^3 where x is
  // distance meters
  private static final double[] LOOKUP_TABLE_HOOD_POLYNOMIAL =
      new double[] {94.38932795, -16.07376701, 5.44989017, -0.73094887};
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
  private static final double LEAD_SCALE = 0.9;

  private static final double RPM_TOLERANCE = 50.0;
  private static final double STABLE_RPM_TIME_SEC = 0.15;
  private static final double RECOVERY_TIME_SEC = 0.25;

  private double lastShotTimestamp = -Double.MAX_VALUE;
  private double rpmStableSince = -Double.MAX_VALUE;
  private double lastCommandedRpm = 0.0;
  private double lastCommandTimestamp = -Double.MAX_VALUE;

  static {
    // TODO: Replace with calibrated distance->shot params (meters, RPM, hood angle deg)
    addShotParams(1.516, Units.radiansPerSecondToRotationsPerMinute(350), 80.0);
    addShotParams(2.439, Units.radiansPerSecondToRotationsPerMinute(375), 77.0);
    addShotParams(3.6097, Units.radiansPerSecondToRotationsPerMinute(380), 73.0);
    addShotParams(4.569, Units.radiansPerSecondToRotationsPerMinute(410), 65.0);

    // Match LauncherCalculator time-of-flight map so lead displacement is equally aggressive.
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(4.55, 1.12);
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
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * LEAD_PHASE_DELAY_SEC,
                robotRelativeSpeeds.vyMetersPerSecond * LEAD_PHASE_DELAY_SEC,
                robotRelativeSpeeds.omegaRadiansPerSecond * LEAD_PHASE_DELAY_SEC));

    Translation2d target2d = fixedTarget.toTranslation2d();
    Pose2d launcherPosition =
        estimatedRobotPose.transformBy(
            new Transform2d(
                new Translation2d(Mechanical.shooterPose.getX(), Mechanical.shooterPose.getY()),
                robotPose.getRotation()));
    double launcherToTargetDistance = target2d.getDistance(launcherPosition.getTranslation());

    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());
    double launcherVelocityX = fieldRelative.vxMetersPerSecond;
    double launcherVelocityY = fieldRelative.vyMetersPerSecond;

    double timeOfFlightSec = getNaiveTimeOfFlightSec(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < LEAD_LOOKAHEAD_ITERATIONS; i++) {
      timeOfFlightSec = getNaiveTimeOfFlightSec(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocityX * timeOfFlightSec * LEAD_SCALE;
      double offsetY = launcherVelocityY * timeOfFlightSec * LEAD_SCALE;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target2d.getDistance(lookaheadPose.getTranslation());
    }

    Translation2d virtualTarget2d =
        target2d.minus(
            new Translation2d(
                launcherVelocityX * timeOfFlightSec * LEAD_SCALE,
                launcherVelocityY * timeOfFlightSec * LEAD_SCALE));
    Translation2d directVector2d = target2d.minus(launcherPosition.getTranslation());
    Translation2d leadVector2d = virtualTarget2d.minus(launcherPosition.getTranslation());

    double yaw = directVector2d.getAngle().getRadians();
    double finalYaw = leadVector2d.getAngle().getRadians();

    ShotParams ledStaticParams = getStaticShootingParams(lookaheadLauncherToTargetDistance);
    double ledExitVelocity =
        calculateExitVelocityMetersPerSecondFromRpm(ledStaticParams.flywheelRpm);
    double pitchStatic = ledStaticParams.hoodAngleRad;
    double minPitch = Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg);
    double maxPitch = Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg);
    double clampedPitchStatic = MathUtil.clamp(pitchStatic, minPitch, maxPitch);

    Translation3d shooterPosition =
        new Pose3d(estimatedRobotPose).plus(Mechanical.shooterPose).getTranslation();
    Translation3d virtualTarget3d =
        new Translation3d(virtualTarget2d.getX(), virtualTarget2d.getY(), fixedTarget.getZ());
    Translation3d shooterToVirtualTarget = virtualTarget3d.minus(shooterPosition);

    Translation3d vStatic =
        new Translation3d(
            ledExitVelocity * Math.cos(clampedPitchStatic) * Math.cos(finalYaw),
            ledExitVelocity * Math.cos(clampedPitchStatic) * Math.sin(finalYaw),
            ledExitVelocity * Math.sin(clampedPitchStatic));

    Translation3d vRobot = new Translation3d(launcherVelocityX, launcherVelocityY, 0.0);

    Translation3d vFinal = vStatic.minus(vRobot);
    double finalPitch = Math.atan2(vStatic.getZ(), vStatic.toTranslation2d().getNorm());
    double clampedFinalPitch = MathUtil.clamp(finalPitch, minPitch, maxPitch);
    double finalExitVelocity = vFinal.getNorm();
    double rawRpm = calculateFlywheelRpmFromExitVelocity(finalExitVelocity);
    double limitedRpm = limitRpm(rawRpm, Timer.getFPGATimestamp());

    double yawErrorMeters =
        lookaheadLauncherToTargetDistance * Math.abs(MathUtil.angleModulus(finalYaw - yaw));
    double pitchErrorMeters =
        lookaheadLauncherToTargetDistance * Math.abs(clampedFinalPitch - clampedPitchStatic);

    Pose3d virtualTargetPose =
        new Pose3d(
            virtualTarget3d.getX(),
            virtualTarget3d.getY(),
            virtualTarget3d.getZ(),
            new Rotation3d());

    Logger.recordOutput("ShootingManager/LeadYawDeg", Units.radiansToDegrees(finalYaw - yaw));
    Logger.recordOutput("ShootingManager/FinalPitchDeg", Units.radiansToDegrees(clampedFinalPitch));
    Logger.recordOutput(
        "ShootingManager/StaticPitchDeg", Units.radiansToDegrees(clampedPitchStatic));
    Logger.recordOutput(
        "ShootingManager/LookaheadDistanceMeters", lookaheadLauncherToTargetDistance);
    Logger.recordOutput("ShootingManager/TimeOfFlightSec", timeOfFlightSec);
    Logger.recordOutput("ShootingManager/LeadScale", LEAD_SCALE);
    Logger.recordOutput("ShootingManager/VirtualTargetX", virtualTarget2d.getX());
    Logger.recordOutput("ShootingManager/VirtualTargetY", virtualTarget2d.getY());
    Logger.recordOutput("ShootingManager/VirtualTarget", virtualTargetPose);
    Logger.recordOutput("ShootingManager/RawRadPerSec", rawRpm * (2 * Math.PI / 60));
    Logger.recordOutput("ShootingManager/LimitedRadPerSec", limitedRpm * (2 * Math.PI / 60));
    Logger.recordOutput("ShootingManager/HoodPitchDeg", Units.radiansToDegrees(clampedFinalPitch));

    return new ShotSolution(
        Rotation2d.fromRadians(finalYaw),
        clampedFinalPitch,
        limitedRpm,
        lookaheadLauncherToTargetDistance,
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

  private double getNaiveTimeOfFlightSec(double distanceMeters) {
    Double mapped = timeOfFlightMap.get(distanceMeters);
    if (mapped != null && Double.isFinite(mapped) && mapped > 0.0) {
      return mapped;
    }

    double rpm = getStaticShootingParams(distanceMeters).flywheelRpm;
    double exitVelocity = calculateExitVelocityMetersPerSecondFromRpm(rpm);
    return distanceMeters / Math.max(1e-6, exitVelocity);
  }
}

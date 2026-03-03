package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants.Mechanical;
import frc.robot.subsystems.shooter.shooterUtil.ShootingCalculator;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingManager {
  public static final Translation3d[] CAMERA_X_Y_Z_OFFSETS =
      new Translation3d[] {
        new Translation3d(
            Units.inchesToMeters(12.066),
            Units.inchesToMeters(11.906),
            Units.inchesToMeters(8.355)),
        new Translation3d(
            Units.inchesToMeters(12.066),
            Units.inchesToMeters(-11.906),
            Units.inchesToMeters(8.355)),
        new Translation3d(
            Units.inchesToMeters(7.0), Units.inchesToMeters(0.0), Units.inchesToMeters(10.0))
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

  private static final double POSE_BUFFER_SECONDS = 1.0;
  private static final double ODOM_BASE_FOM = 1.0;
  private static final double ODOM_DRIFT_FOM_PER_SEC = 0.15;
  private static final double COLLISION_FOM_SPIKE = 5.0;
  private static final double VISION_BASE_FOM = 1.5;
  private static final double VISION_ROT_FOM_PER_RAD = 0.4;
  private static final double VISION_OFFSET_FOM_PER_DEG = 0.05;
  private static final double VISION_DIST_FOM_PER_M = 0.1;
  private static final double GRAVITY_MPS2 = 9.80665;
  private static final double MAX_FLYWHEEL_RPM = 6271.0;
  private static final double RPM_RATE_LIMIT = 600.0;

  private static final double RPM_TOLERANCE = 50.0;
  private static final double STABLE_RPM_TIME_SEC = 0.15;
  private static final double RECOVERY_TIME_SEC = 0.25;

  private final Deque<PoseSample> poseBuffer = new ArrayDeque<>();
  private Pose2d estimatedPose = Pose2d.kZero;
  private double odomFoM = ODOM_BASE_FOM;
  private double lastVisionTimestamp = 0.0;
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

  public void updateFromSuppliers() {
    if (poseSupplier == null || speedsSupplier == null) {
      return;
    }
    updateOdometry(poseSupplier.get(), speedsSupplier.get(), Timer.getFPGATimestamp());
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSec) {
    double angularVelocity = 0.0;
    if (speedsSupplier != null) {
      angularVelocity = speedsSupplier.get().omegaRadiansPerSecond;
    } else if (headingSupplier != null) {
      angularVelocity = estimateAngularVelocity(headingSupplier.get(), timestampSec);
    }
    addVisionObservation(visionPose, timestampSec, angularVelocity, 0.0, 0, 0.0);
  }

  public void updateOdometry(Pose2d pose, ChassisSpeeds robotRelativeSpeeds, double timestampSec) {
    estimatedPose = pose;
    odomFoM = ODOM_BASE_FOM + (timestampSec - lastVisionTimestamp) * ODOM_DRIFT_FOM_PER_SEC;
    poseBuffer.addLast(new PoseSample(pose, robotRelativeSpeeds, timestampSec));
    trimPoseBuffer(timestampSec);
  }

  public void recordCollision(double accelG) {
    if (accelG > 2.0) {
      odomFoM += COLLISION_FOM_SPIKE;
    }
  }

  public void recordSkid(boolean skidding) {
    if (skidding) {
      odomFoM += COLLISION_FOM_SPIKE;
    }
  }

  // public void addVisionObservation(
  //     VisionInputs inputs, double robotAngularVelocityRadPerSec, double targetYawDeg) {
  //   addVisionObservation(
  //       inputs.pose,
  //       inputs.timestamp,
  //       robotAngularVelocityRadPerSec,
  //       targetYawDeg,
  //       inputs.tagCount,
  //       inputs.averageDistance);
  // }

  public void addVisionObservation(
      Pose2d visionPose,
      double timestampSec,
      double robotAngularVelocityRadPerSec,
      double targetYawDeg,
      int tagCount,
      double avgDistance) {
    PoseSample sample = getPoseSampleAt(timestampSec);
    if (sample == null) {
      return;
    }

    double visionFoM =
        VISION_BASE_FOM
            + Math.abs(robotAngularVelocityRadPerSec) * VISION_ROT_FOM_PER_RAD
            + Math.abs(targetYawDeg) * VISION_OFFSET_FOM_PER_DEG
            + Math.abs(avgDistance) * VISION_DIST_FOM_PER_M;

    if (tagCount <= 0) {
      visionFoM += 2.0;
    }

    Pose2d fusedAtTimestamp = blendPoses(sample.pose, visionPose, odomFoM, visionFoM);
    applyPoseCorrection(timestampSec, new Transform2d(sample.pose, fusedAtTimestamp));
    lastVisionTimestamp = timestampSec;
    odomFoM = ODOM_BASE_FOM;

    Logger.recordOutput("ShootingManager/FoM/Odometry", odomFoM);
    Logger.recordOutput("ShootingManager/FoM/Vision", visionFoM);
    Logger.recordOutput("ShootingManager/Pose/Fused", estimatedPose);
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public double getPoseFoM() {
    return odomFoM;
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
        ShootingCalculator.calculateExitVelocityMetersPerSecondFromRpm(staticParams.flywheelRpm);

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
    double rawRpm = ShootingCalculator.calculateFlywheelRpmFromExitVelocity(finalExitVelocity);
    double limitedRpm = limitRpm(rawRpm, Timer.getFPGATimestamp());

    double yawErrorMeters = distanceMeters * Math.abs(MathUtil.angleModulus(finalYaw - yaw));
    double pitchErrorMeters = distanceMeters * Math.abs(clampedFinalPitch - clampedPitchStatic);

    Logger.recordOutput("ShootingManager/LeadYawDeg", Units.radiansToDegrees(finalYaw - yaw));
    Logger.recordOutput("ShootingManager/FinalPitchDeg", Units.radiansToDegrees(clampedFinalPitch));
    Logger.recordOutput(
        "ShootingManager/StaticPitchDeg", Units.radiansToDegrees(clampedPitchStatic));
    Logger.recordOutput("ShootingManager/RawRpm", rawRpm);
    Logger.recordOutput("ShootingManager/LimitedRpm", limitedRpm);
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

    double exitVelocity =
        ShootingCalculator.calculateExitVelocityMetersPerSecondFromRpm(solution.flywheelRpm);
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
      ShotSolution solution, double poseFoMThreshold, boolean rpmStable, double timestampSec) {
    boolean poseTrusted = odomFoM <= poseFoMThreshold;
    boolean withinAngle = solution.yawWithinTolerance && solution.pitchWithinTolerance;
    boolean recovered = timestampSec - lastShotTimestamp >= RECOVERY_TIME_SEC;
    return poseTrusted && withinAngle && rpmStable && recovered;
  }

  public double getCurrentTime() {
    return Timer.getFPGATimestamp();
  }

  private void trimPoseBuffer(double timestampSec) {
    while (!poseBuffer.isEmpty()
        && timestampSec - poseBuffer.peekFirst().timestampSec > POSE_BUFFER_SECONDS) {
      poseBuffer.removeFirst();
    }
  }

  private PoseSample getPoseSampleAt(double timestampSec) {
    if (poseBuffer.isEmpty()) {
      return null;
    }

    PoseSample previous = null;
    for (PoseSample sample : poseBuffer) {
      if (sample.timestampSec >= timestampSec) {
        if (previous == null) {
          return sample;
        }
        double t =
            (timestampSec - previous.timestampSec)
                / Math.max(1e-6, sample.timestampSec - previous.timestampSec);
        Pose2d interpolated = previous.pose.interpolate(sample.pose, t);
        return new PoseSample(interpolated, sample.robotRelativeSpeeds, timestampSec);
      }
      previous = sample;
    }
    return poseBuffer.peekLast();
  }

  private void applyPoseCorrection(double timestampSec, Transform2d correction) {
    if (correction == null) {
      return;
    }
    Deque<PoseSample> corrected = new ArrayDeque<>();
    for (PoseSample sample : poseBuffer) {
      Pose2d pose = sample.pose;
      if (sample.timestampSec >= timestampSec) {
        pose = pose.transformBy(correction);
      }
      corrected.addLast(new PoseSample(pose, sample.robotRelativeSpeeds, sample.timestampSec));
    }
    poseBuffer.clear();
    poseBuffer.addAll(corrected);
    if (!poseBuffer.isEmpty()) {
      estimatedPose = poseBuffer.peekLast().pose;
    } else {
      estimatedPose = estimatedPose.transformBy(correction);
    }
  }

  private Pose2d blendPoses(Pose2d odomPose, Pose2d visionPose, double odomFom, double visionFom) {
    double odomWeight = 1.0 / Math.max(1e-6, odomFom);
    double visionWeight = 1.0 / Math.max(1e-6, visionFom);
    double totalWeight = odomWeight + visionWeight;

    double x = (odomPose.getX() * odomWeight + visionPose.getX() * visionWeight) / totalWeight;
    double y = (odomPose.getY() * odomWeight + visionPose.getY() * visionWeight) / totalWeight;
    double rot =
        (odomPose.getRotation().getRadians() * odomWeight
                + visionPose.getRotation().getRadians() * visionWeight)
            / totalWeight;
    return new Pose2d(x, y, Rotation2d.fromRadians(rot));
  }

  private static class PoseSample {
    private final Pose2d pose;
    private final ChassisSpeeds robotRelativeSpeeds;
    private final double timestampSec;

    private PoseSample(Pose2d pose, ChassisSpeeds robotRelativeSpeeds, double timestampSec) {
      this.pose = pose;
      this.robotRelativeSpeeds = robotRelativeSpeeds;
      this.timestampSec = timestampSec;
    }
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
}

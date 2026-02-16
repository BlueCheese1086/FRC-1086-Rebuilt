package frc.robot.subsystems.shooter.shooterUtil;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shooter.ShooterConstants.Mechanical.flywheelRadius;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants.Mechanical;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Hub;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculator {

  private static final double GRAVITY_MPS2 = FieldConstants.gravitationalAcceleration;
  private static final double TARGET_HEIGHT_DELTA_METERS = Units.inchesToMeters(72);
  private static final int MOVING_SOLUTION_ITERATIONS = 4;
  private static final int HOOD_ANGLE_SAMPLES = 25;
  private static final double STATIONARY_EXIT_VELOCITY_TOLERANCE_MPS = 0.25;

  private static double lastHoodAngleRad = Units.degreesToRadians(67.0);
  private static double lastFlywheelRpm = 0.0;

  public static class ShootingSolution {
    public final double hoodAngleRad;
    public final double flywheelRpm;
    public final double exitVelocityMps;
    public final double distanceMeters;
    public final boolean valid;

    private ShootingSolution(
        double hoodAngleRad,
        double flywheelRpm,
        double exitVelocityMps,
        double distanceMeters,
        boolean valid) {
      this.hoodAngleRad = hoodAngleRad;
      this.flywheelRpm = flywheelRpm;
      this.exitVelocityMps = exitVelocityMps;
      this.distanceMeters = distanceMeters;
      this.valid = valid;
    }
  }

  public static double calculateExitAngularVelocityRadPerSec(
      double distanceMeters, double hoodAngleRad) {
    double exitVelocityMps = calculateMetersPerSecond(distanceMeters, hoodAngleRad);
    if (exitVelocityMps <= 0.0) {
      return 0.0;
    }
    return exitVelocityMps / flywheelRadius.in(Meters);
  }

  public static double calculateExitVelocityMetersPerSecondFromRpm(double flywheelRpm) {
    double omegaRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheelRpm);
    return omegaRadPerSec * flywheelRadius.in(Meters);
  }

  public static double calculateFlywheelRpmFromExitVelocity(double exitVelocityMps) {
    if (exitVelocityMps <= 0.0) {
      return 0.0;
    }
    double omegaRadPerSec = exitVelocityMps / flywheelRadius.in(Meters);
    return Units.radiansPerSecondToRotationsPerMinute(omegaRadPerSec);
  }

  public static double calculateFlywheelRpm(double distanceMeters, double hoodAngleRad) {
    double exitVelocityMps = calculateMetersPerSecond(distanceMeters, hoodAngleRad);
    return calculateFlywheelRpmFromExitVelocity(exitVelocityMps);
  }

  public static double calculateMetersPerSecond(double distanceMeters, double hoodAngleRad) {
    double safeDistance = Math.max(distanceMeters, getMinimumDistanceMeters(hoodAngleRad));
    double cos = Math.cos(hoodAngleRad);
    double tan = Math.tan(hoodAngleRad);
    double denom = 2.0 * cos * cos * (safeDistance * tan - TARGET_HEIGHT_DELTA_METERS);
    double product = (GRAVITY_MPS2 * safeDistance * safeDistance) / denom;
    if (!Double.isFinite(product) || product < 0.0) {
      return 0.0;
    }
    return Math.sqrt(product);
  }

  private static double getMinimumDistanceMeters(double hoodAngleRad) {
    double tan = Math.tan(hoodAngleRad);
    if (!Double.isFinite(tan) || Math.abs(tan) < 1e-6) {
      return 0.0;
    }
    return (TARGET_HEIGHT_DELTA_METERS / tan) + 1e-3;
  }

  public static double calculateTimeOfFlight(
      double distanceMeters, double hoodAngleRad, double exitVelocityMps) {
    Logger.recordOutput("Shooting/TOF_InputDistanceMeters", distanceMeters);
    double backDistanceMeters = Units.inchesToMeters(0);
    double totalHorizontalDistMeters = distanceMeters + backDistanceMeters;

    double vx = exitVelocityMps * Math.cos(hoodAngleRad);

    if (!Double.isFinite(vx) || vx <= 1e-6) {
      return 1.0; // Fallback to 1 second if something is wrong
    }

    double timeSeconds = totalHorizontalDistMeters / vx;

    Logger.recordOutput("Shooting/TOF_HorizontalDistMeters", totalHorizontalDistMeters);
    Logger.recordOutput("Shooting/TOF_Theta", Math.toDegrees(hoodAngleRad));
    Logger.recordOutput("Shooting/TOF_VelocityMPS", exitVelocityMps);
    Logger.recordOutput("Shooting/TOF_VxMPS", vx);
    Logger.recordOutput("Shooting/TOF_Calculated", timeSeconds);

    return timeSeconds;
  }

  public static double calculateTimeOfFlight(double distanceMeters, double hoodAngleRad) {
    double exitVelocityMps = calculateMetersPerSecond(distanceMeters, hoodAngleRad);
    return calculateTimeOfFlight(distanceMeters, hoodAngleRad, exitVelocityMps);
  }

  public static ShootingSolution calculateMovingSolution(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      double hoodMinRad,
      double hoodMaxRad,
      double flywheelMinRpm,
      double flywheelMaxRpm,
      double rpmChangeWeight,
      double hoodChangeWeight) {
    double minAngle = Math.min(hoodMinRad, hoodMaxRad);
    double maxAngle = Math.max(hoodMinRad, hoodMaxRad);
    if (!Double.isFinite(minAngle) || !Double.isFinite(maxAngle) || minAngle == maxAngle) {
      minAngle = hoodMinRad;
      maxAngle = hoodMaxRad;
    }

    ShootingSolution best = null;
    double bestCost = Double.POSITIVE_INFINITY;
    for (int i = 0; i < HOOD_ANGLE_SAMPLES; i++) {
      double t = (double) i / (HOOD_ANGLE_SAMPLES - 1);
      double hoodAngle = minAngle + (maxAngle - minAngle) * t;
      ShootingSolution candidate =
          solveForMovingAngle(robotPose, robotSpeeds, hoodAngle, flywheelMinRpm, flywheelMaxRpm);
      if (candidate.valid) {
        double rpmDelta = Math.abs(candidate.flywheelRpm - lastFlywheelRpm);
        double hoodDelta = Math.abs(candidate.hoodAngleRad - lastHoodAngleRad);
        double cost = (rpmDelta * rpmChangeWeight) + (hoodDelta * hoodChangeWeight);
        if (best == null || cost < bestCost) {
          best = candidate;
          bestCost = cost;
        }
      }
    }

    if (best == null) {
      double fallbackAngle = (minAngle + maxAngle) * 0.5;
      double distanceMeters =
          robotPose.getTranslation().getDistance(Hub.topCenterPoint.toTranslation2d());
      double exitVelocityMps = calculateMetersPerSecond(distanceMeters, fallbackAngle);
      double rpm = calculateFlywheelRpmFromExitVelocity(exitVelocityMps);
      double clampedRpm = clamp(rpm, flywheelMinRpm, flywheelMaxRpm);
      best =
          new ShootingSolution(fallbackAngle, clampedRpm, exitVelocityMps, distanceMeters, false);
    }

    updateLastSolution(best);
    Logger.recordOutput("Shooting/Solution/HoodAngleDeg", Math.toDegrees(best.hoodAngleRad));
    Logger.recordOutput("Shooting/Solution/FlywheelRpm", best.flywheelRpm);
    Logger.recordOutput("Shooting/Solution/ExitVelocityMps", best.exitVelocityMps);
    Logger.recordOutput("Shooting/Solution/DistanceMeters", best.distanceMeters);
    Logger.recordOutput("Shooting/Solution/Valid", best.valid);

    return best;
  }

  private static ShootingSolution solveForMovingAngle(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      double hoodAngleRad,
      double flywheelMinRpm,
      double flywheelMaxRpm) {
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose3d shooterPose = robotPose3d.plus(Mechanical.shooterPose);
    double distanceMeters =
        shooterPose
            .getTranslation()
            .toTranslation2d()
            .getDistance(Hub.topCenterPoint.toTranslation2d());
    double exitVelocityMps = 0.0;

    for (int i = 0; i < MOVING_SOLUTION_ITERATIONS; i++) {
      exitVelocityMps = calculateMetersPerSecond(distanceMeters, hoodAngleRad);
      if (!Double.isFinite(exitVelocityMps) || exitVelocityMps <= 0.0) {
        return new ShootingSolution(hoodAngleRad, 0.0, 0.0, distanceMeters, false);
      }
      Pose3d virtualTarget =
          virtualHub.getVirtualTargetWithExitVelocity(
              robotPose3d, robotSpeeds, hoodAngleRad, exitVelocityMps);
      distanceMeters =
          shooterPose
              .getTranslation()
              .toTranslation2d()
              .getDistance(virtualTarget.getTranslation().toTranslation2d());
      if (!Double.isFinite(distanceMeters)) {
        return new ShootingSolution(hoodAngleRad, 0.0, 0.0, distanceMeters, false);
      }
    }

    double flywheelRpm = calculateFlywheelRpmFromExitVelocity(exitVelocityMps);
    boolean valid =
        Double.isFinite(flywheelRpm)
            && flywheelRpm >= flywheelMinRpm
            && flywheelRpm <= flywheelMaxRpm;
    return new ShootingSolution(
        hoodAngleRad,
        clamp(flywheelRpm, flywheelMinRpm, flywheelMaxRpm),
        exitVelocityMps,
        distanceMeters,
        valid);
  }

  private static double clamp(double value, double min, double max) {
    if (!Double.isFinite(value)) {
      return min;
    }
    return Math.max(min, Math.min(max, value));
  }

  public static ShootingSolution calculateStationarySolution(
      Pose2d robotPose, double hoodMinRad, double hoodMaxRad, double fixedFlywheelRpm) {
    double minAngle = Math.min(hoodMinRad, hoodMaxRad);
    double maxAngle = Math.max(hoodMinRad, hoodMaxRad);
    if (!Double.isFinite(minAngle) || !Double.isFinite(maxAngle) || minAngle == maxAngle) {
      minAngle = hoodMinRad;
      maxAngle = hoodMaxRad;
    }

    double distanceMeters =
        robotPose.getTranslation().getDistance(Hub.topCenterPoint.toTranslation2d());
    double desiredExitVelocity = calculateExitVelocityMetersPerSecondFromRpm(fixedFlywheelRpm);

    double bestAngle = (minAngle + maxAngle) * 0.5;
    double bestExitVelocity = 0.0;
    double bestError = Double.POSITIVE_INFINITY;

    for (int i = 0; i < HOOD_ANGLE_SAMPLES; i++) {
      double t = (double) i / (HOOD_ANGLE_SAMPLES - 1);
      double hoodAngle = minAngle + (maxAngle - minAngle) * t;
      double exitVelocity = calculateMetersPerSecond(distanceMeters, hoodAngle);
      if (!Double.isFinite(exitVelocity) || exitVelocity <= 0.0) {
        continue;
      }
      double error = Math.abs(exitVelocity - desiredExitVelocity);
      if (error < bestError) {
        bestError = error;
        bestAngle = hoodAngle;
        bestExitVelocity = exitVelocity;
      }
    }

    boolean valid =
        Double.isFinite(desiredExitVelocity)
            && desiredExitVelocity > 0.0
            && bestError <= STATIONARY_EXIT_VELOCITY_TOLERANCE_MPS;
    ShootingSolution solution =
        new ShootingSolution(bestAngle, fixedFlywheelRpm, bestExitVelocity, distanceMeters, valid);
    updateLastSolution(solution);
    return solution;
  }

  private static void updateLastSolution(ShootingSolution solution) {
    if (solution == null) {
      return;
    }
    lastHoodAngleRad = solution.hoodAngleRad;
    lastFlywheelRpm = solution.flywheelRpm;
  }

  public static class virtualHub {
    public static Pose3d getVirtualTargetWithExitVelocity(
        Pose3d robotPose, ChassisSpeeds robotSpeeds, double hoodAngleRad, double exitVelocityMps) {
      return getVirtualTargetWithExitVelocity(
          robotPose, robotSpeeds, Hub.topCenterPoint, hoodAngleRad, exitVelocityMps);
    }

    public static Pose3d getVirtualTargetWithExitVelocity(
        Pose3d robotPose,
        ChassisSpeeds robotSpeeds,
        Translation3d actualGoal,
        double hoodAngleRad,
        double exitVelocityMps) {

      Pose3d shooterPose = robotPose.plus(Mechanical.shooterPose);
      Translation3d shooterPos = shooterPose.getTranslation();

      ChassisSpeeds fieldRelativeSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, shooterPose.toPose2d().getRotation());
      Translation3d robotVelVec =
          new Translation3d(
              fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond, 0.0);

      double distanceToReal =
          shooterPos.toTranslation2d().getDistance(actualGoal.toTranslation2d());

      double tof =
          ShootingCalculator.calculateTimeOfFlight(distanceToReal, hoodAngleRad, exitVelocityMps);

      Translation3d virtualTarget = actualGoal;

      for (int i = 0; i < 4; i++) {
        virtualTarget = actualGoal.minus(robotVelVec.times(tof));
        double virtualDist =
            shooterPos.toTranslation2d().getDistance(virtualTarget.toTranslation2d());
        if (virtualDist < 0.0) {
          virtualDist = 0.0;
        }
        tof = ShootingCalculator.calculateTimeOfFlight(virtualDist, hoodAngleRad, exitVelocityMps);
      }
      Logger.recordOutput("VirtualTargetPose", new Pose3d(virtualTarget, new Rotation3d()));
      return new Pose3d(virtualTarget, new Rotation3d());
    }

    public static Pose3d getVirtualTarget(
        Pose3d robotPose, ChassisSpeeds robotSpeeds, double hoodAngleRad, double flywheelRpm) {
      double exitVelocityMps = calculateExitVelocityMetersPerSecondFromRpm(flywheelRpm);
      return getVirtualTargetWithExitVelocity(
          robotPose, robotSpeeds, hoodAngleRad, exitVelocityMps);
    }

    public static double getDistanceToVirtualTargetWithExitVelocity(
        Pose3d robotPose, ChassisSpeeds robotSpeeds, double hoodAngleRad, double exitVelocityMps) {
      Pose3d virtualTargetPose =
          getVirtualTargetWithExitVelocity(robotPose, robotSpeeds, hoodAngleRad, exitVelocityMps);
      return robotPose
          .getTranslation()
          .toTranslation2d()
          .getDistance(virtualTargetPose.getTranslation().toTranslation2d());
    }

    public static double getDistanceToVirtualTarget(
        Pose3d robotPose, ChassisSpeeds robotSpeeds, double hoodAngleRad, double flywheelRpm) {
      double exitVelocityMps = calculateExitVelocityMetersPerSecondFromRpm(flywheelRpm);
      return getDistanceToVirtualTargetWithExitVelocity(
          robotPose, robotSpeeds, hoodAngleRad, exitVelocityMps);
    }
  }
}

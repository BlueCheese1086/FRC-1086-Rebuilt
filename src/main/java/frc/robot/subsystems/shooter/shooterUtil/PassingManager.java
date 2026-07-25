// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PassingManager {
  private static PassingManager instance;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
  private final LinearFilter driveAngleFilter = LinearFilter.movingAverage((int) (0.8 / 0.02));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;
  private Translation2d target = new Translation2d();

  public static PassingManager getInstance() {
    if (instance == null) instance = new PassingManager();
    return instance;
  }

  public record PassingParams(
      boolean isValid,
      Rotation2d driveAngle,
      Rotation2d driveAngleNoLookahead,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight) {}

  // Cache parameters
  private PassingParams latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 3.0;
    maxDistance = 18.0;
    phaseDelay = 0.03;

    hoodAngleMap.put(14.5, Rotation2d.fromDegrees(54.0));
    hoodAngleMap.put(2.323, Rotation2d.fromDegrees(54.0));
    hoodAngleMap.put(2.762, Rotation2d.fromDegrees(54.0));

    flywheelSpeedMap.put(14.5, 600.0);
    flywheelSpeedMap.put(11.77, 525.0);
    flywheelSpeedMap.put(8.945, 450.0);
    flywheelSpeedMap.put(6.7, 370.0);

    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public PassingParams getParameters(
      Supplier<Pose2d> drivePose,
      Supplier<ChassisSpeeds> robotRelativeSpeeds,
      Supplier<Rotation2d> heading) {
    // if (latestParameters != null) {
    // return latestParameters;
    // }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = drivePose.get();
    ChassisSpeeds robotRelativeVelocity = robotRelativeSpeeds.get();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from launcher to target
    if (drivePose.get().getY()
        >= (FieldConstants.fieldWidth / 2) + (FieldConstants.Hub.width / 2)) {
      target = AllianceFlipUtil.apply(FieldConstants.Tower.leftBackPose.getTranslation());
    } else if (drivePose.get().getY()
        <= (FieldConstants.fieldWidth / 2) - (FieldConstants.Hub.width / 2)) {
      target = AllianceFlipUtil.apply(FieldConstants.Tower.rightBackPose.getTranslation());
    }

    Pose2d shooterPose = Shooter.shooterPose(drivePose);

    Pose2d launcherPosition =
        estimatedPose.transformBy(
            new Transform2d(
                new Translation2d(shooterPose.getX(), shooterPose.getY()),
                drivePose.get().getRotation()));
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    // This isn't actually the launcherVelocity given it won't account for angular
    // velocity of robot
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, heading.get());
    double launcherVelocityX = fieldRelative.vxMetersPerSecond;
    double launcherVelocityY = fieldRelative.vyMetersPerSecond;

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight = timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocityX * timeOfFlight;
      double offsetY = launcherVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    Rotation2d driveAngleNoLookahead =
        target.minus(launcherPosition.getTranslation()).getAngle().plus(Rotation2d.kPi);
    Rotation2d driveAngle =
        target
            .minus(
                new Translation2d(
                    launcherVelocityX * timeOfFlight, launcherVelocityY * timeOfFlight))
            .minus(launcherPosition.getTranslation())
            .getAngle();
    // target.minus(lookaheadPose.getTranslation()).getAngle().plus(Rotation2d.kPi);
    double hoodAngle = hoodAngleMap.get(lookaheadLauncherToTargetDistance).getDegrees();

    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / 0.02);
    lastDriveAngle = driveAngle;
    latestParameters =
        new PassingParams(
            lookaheadLauncherToTargetDistance >= minDistance
                && lookaheadLauncherToTargetDistance <= maxDistance,
            driveAngle,
            driveAngleNoLookahead,
            driveVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelSpeedMap.get(lookaheadLauncherToTargetDistance),
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);
    Logger.recordOutput(
        "SOTM/Virtual Hub",
        new Pose2d(
            target.minus(
                new Translation2d(
                    launcherVelocityX * timeOfFlight, launcherVelocityY * timeOfFlight)),
            new Rotation2d()));

    return latestParameters;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}

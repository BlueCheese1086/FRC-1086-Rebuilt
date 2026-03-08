// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterUtil;

import static frc.robot.subsystems.shooter.ShooterConstants.Mechanical.shooterPose;

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
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LauncherCalculator {
  private static LauncherCalculator instance;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
  private final LinearFilter driveAngleFilter = LinearFilter.movingAverage((int) (0.8 / 0.02));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LauncherCalculator getInstance() {
    if (instance == null) instance = new LauncherCalculator();
    return instance;
  }

  public record LaunchingParameters(
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
  private LaunchingParameters latestParameters = null;

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
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;

    hoodAngleMap.put(2.43, Rotation2d.fromDegrees(68.0));
    hoodAngleMap.put(1.581, Rotation2d.fromDegrees(78.0));
    hoodAngleMap.put(1.94, Rotation2d.fromDegrees(74.0));
    hoodAngleMap.put(2.414, Rotation2d.fromDegrees(72.0));
    hoodAngleMap.put(2.85, Rotation2d.fromDegrees(67.0));
    hoodAngleMap.put(3.11, Rotation2d.fromDegrees(64.0));

    flywheelSpeedMap.put(1.4478, Units.rotationsPerMinuteToRadiansPerSecond(3351.0));
    flywheelSpeedMap.put(1.8, Units.rotationsPerMinuteToRadiansPerSecond(3351.803));
    flywheelSpeedMap.put(2.054, Units.rotationsPerMinuteToRadiansPerSecond(3351.803));
    flywheelSpeedMap.put(3.048, Units.rotationsPerMinuteToRadiansPerSecond(3437.76));
    flywheelSpeedMap.put(5.334, Units.rotationsPerMinuteToRadiansPerSecond(3819.71));

    // timeOfFlightMap.put(5.68, 1.16);
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

  public LaunchingParameters getParameters(
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
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
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
        new LaunchingParameters(
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

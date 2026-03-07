// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter.shooterUtil;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LauncherCalculator {
  private static LauncherCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
  private final LinearFilter driveAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LauncherCalculator getInstance() {
    if (instance == null) instance = new LauncherCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double flywheelIdleSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    passingMinDistance = 0.0;
    passingMaxDistance = 12.0;
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
    // flywheelSpeedMap.put(2.47, 170.0);
    // flywheelSpeedMap.put(2.70, 170.0);
    // flywheelSpeedMap.put(2.94, 175.0);
    // flywheelSpeedMap.put(3.48, 175.0);
    // flywheelSpeedMap.put(3.92, 180.0);
    // flywheelSpeedMap.put(4.35, 185.0);
    // flywheelSpeedMap.put(4.84, 190.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(38.0));

    passingFlywheelSpeedMap.put(5.46, 160.0);
    passingFlywheelSpeedMap.put(6.62, 180.0);
    passingFlywheelSpeedMap.put(7.80, 200.0);

    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters(
      Supplier<Pose2d> drivePose, Supplier<ChassisSpeeds> roboRelativeSpeed) {
    boolean passing =
        AllianceFlipUtil.applyX(drivePose.get().getX()) > FieldConstants.LinesVertical.hubCenter;
    // if (latestParameters != null) {
    // return latestParameters;
    // }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = drivePose.get();
    ChassisSpeeds robotRelativeVelocity = roboRelativeSpeed.get();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d launcherPosition = Shooter.getShooterPoses(estimatedPose)[1].toPose2d();
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, drivePose.get().getRotation());
    var robotAngle = drivePose.get().getRotation();
    ChassisSpeeds launcherVelocity =
        transformVelocity(
            robotVelocity,
            ShooterConstants.Mechanical.shooterPose.getTranslation().toTranslation2d(),
            robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(
            new Transform2d(
                ShooterConstants.Mechanical.shooterPose.getTranslation().toTranslation2d(),
                estimatedPose.getRotation()));
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getDegrees()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getDegrees();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / 0.02);
    lastDriveAngle = driveAngle;

    double flywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    Rotation2d hubAngle =
        target
            .minus(
                new Translation2d(
                    launcherVelocity.vxMetersPerSecond * timeOfFlight,
                    launcherVelocity.vyMetersPerSecond * timeOfFlight))
            .minus(launcherPosition.getTranslation())
            .getAngle();

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            FieldConstants.inFieldBounds(lookaheadRobotPose),
            hubAngle,
            driveVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelVelocity,
            350.0,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);
    Logger.recordOutput("LauncherCalculator/Estimated Pose", estimatedPose);

    return latestParameters;
  }

  public static Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    ShooterConstants.Mechanical.shooterPose.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle
            .plus(hubAngle)
            .plus(ShooterConstants.Mechanical.shooterPose.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(
      Translation2d robotTranslation, Rotation2d robotRotation, boolean forceBlue) {
    // Calculate target
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation,
        getDriveAngleWithLauncherOffset(new Pose2d(robotTranslation, robotRotation), target));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }

  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getY() * currentRotation.getCos()
                    - transform.getX() * currentRotation.getSin()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }
}

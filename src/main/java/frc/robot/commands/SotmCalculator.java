// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.shooterUtil.ShootingCalculator;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SotmCalculator {
  private SotmCalculator() {}

  public static Rotation2d getDesiredRotation(Drive drive, Pose2d targetPose) {
    ShootingCalculator.ShootingSolution solution =
        ShootingCalculator.calculateMovingSolution(
            drive.getPose(),
            drive.getChassisSpeeds(),
            Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg),
            Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg),
            ShooterConstants.Targeting.minRpm,
            ShooterConstants.Targeting.maxRpm,
            ShooterConstants.Targeting.movingRpmChangeWeight,
            ShooterConstants.Targeting.movingHoodChangeWeight);

    Pose3d virtualTarget =
        ShootingCalculator.virtualHub.getVirtualTargetWithExitVelocity(
            new Pose3d(drive.getPose()),
            drive.getChassisSpeeds(),
            solution.hoodAngleRad,
            solution.exitVelocityMps);

    Pose2d virtualTarget2d =
        new Pose2d(virtualTarget.getTranslation().toTranslation2d(), Rotation2d.kZero);
    return DriveCommands.getOrientationToTarget(drive.getPose(), virtualTarget2d);
  }

  private static LoggedTunableNumber adjust = new LoggedTunableNumber("SOTM/Adjust", 0.0);

  public static Supplier<Rotation2d> getTargetOrientationOnMove(
      Shooter shooter,
      Supplier<Pose2d> current,
      Supplier<Pose2d> target,
      Supplier<ChassisSpeeds> speeds) {
    return () -> {
      Pose2d currentPose = current.get();
      Pose2d targetPose = target.get();
      ChassisSpeeds currentSpeeds = speeds.get();

      double latency = 0.1;

      Translation2d velVector =
          new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
      Translation2d futurePose = currentPose.getTranslation().plus(velVector.times(latency));
      Translation2d targetVec = targetPose.getTranslation().minus(futurePose);
      double dist = targetVec.getNorm();
      double idealHorizontalSpeed = ShotCalc.getShotMPS(Meters.of(dist));
      Logger.recordOutput("Target", currentPose.getTranslation().plus(targetVec));
      Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(velVector);

      Logger.recordOutput(
          "Targetting/Shoot On Move Pose",
          new Pose2d(currentPose.getTranslation(), shotVec.getAngle()));
      // shooter.setShooterVelocity(
      //     shotVec.getNorm() / Units.inchesToMeters(2.0) / Math.cos(Math.toRadians(65)));
      Logger.recordOutput("Targetting/Velocity ", currentSpeeds);
      return shotVec.getAngle().plus(Rotation2d.fromDegrees(adjust.getAsDouble()));
      // return translationToTarget.getAngle();
    };
  }
}

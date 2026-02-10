// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.shooterUtil.ShootingCalculator;

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
}

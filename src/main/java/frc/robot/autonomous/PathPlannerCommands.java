// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTransforms;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator;

public class PathPlannerCommands {
  public Command intake(Intake intake) {
    return Commands.runOnce(() -> intake.setPosition(IntakeConstants.Setpoints.deployed));
  }

  public Command aimAndShoot(Drive drive, Shooter shooter, Hood hood, Indexer indexer) {
    return Commands.runOnce(() -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(350.0)))
        .until(shooter::atSetpoint)
        .alongWith(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () ->
                        LauncherCalculator.getInstance()
                            .getParameters(
                                () ->
                                    (new Pose3d(drive.getPose())
                                        .transformBy(ShooterTransforms.centerShooter)
                                        .toPose2d()),
                                drive::getChassisSpeeds,
                                drive::getRotation)
                            .driveAngle()),
                indexer.setVoltage(IndexerConstants.Setpoints.feed),
                shooter.runFeed(ShooterConstants.FeederSetpoints.run.in(Volt))));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class SotmCalculator {
  public static Rotation2d getDesiredRotation(Drive drive, Pose2d target) {
    Translation2d speeds2d =
        new Translation2d(
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation())
                    .vxMetersPerSecond
                * -1,
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation())
                    .vyMetersPerSecond
                * -1);
    Rotation2d estimated =
        DriveCommands.getOrientationToTarget(
            new Pose2d(
                drive.getPose().minus(new Pose2d(speeds2d, Rotation2d.kZero)).getTranslation(),
                Rotation2d.kZero),
            target);
    Rotation2d staticRot = DriveCommands.getOrientationToTarget(drive.getPose(), target);
    double difference = estimated.getRadians() - staticRot.getRadians();
    if (Math.abs(difference) <= Math.PI / 4) {
      return estimated;
    } else {
      return staticRot.plus(Rotation2d.fromRadians(Math.PI / 2 * Math.signum(difference)));
    }
  }
}

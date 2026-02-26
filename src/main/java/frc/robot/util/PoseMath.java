// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class PoseMath {
  public static Pose3d average(Pose3d pose1, Pose3d pose2) {
    return new Pose3d(
        (pose1.getX() + pose2.getX()) / 2,
        (pose1.getY() + pose2.getY()) / 2,
        (pose1.getZ() + pose2.getZ()) / 2,
        new Rotation3d(
            (pose1.getRotation().getX() + pose2.getRotation().getX()) / 2,
            (pose1.getRotation().getY() + pose2.getRotation().getY()) / 2,
            (pose1.getRotation().getZ() + pose2.getRotation().getZ()) / 2));
  }

  public static Pose2d average(Pose2d pose1, Pose2d pose2) {
    return new Pose2d(
        (pose1.getX() + pose2.getX()) / 2,
        (pose1.getY() + pose2.getY()) / 2,
        new Rotation2d((pose1.getRotation().getRadians() + pose2.getRotation().getRadians()) / 2));
  }

  public static Pose2d average(Pose2d... poses) {
    ArrayList<Pose2d> poseList = new ArrayList<Pose2d>(List.of(poses));
    Pose2d finalPose = poseList.remove(poseList.size() - 1);
    while (poseList.size() > 0) {
      finalPose = average(finalPose, poseList.remove(poseList.size() - 1));
    }
    return finalPose;
  }

  public static Rotation2d getOrientationToTarget(Pose2d pose, Pose2d target) {
    Translation2d diff = target.relativeTo(pose).getTranslation();
    return diff.getAngle();
  }

  public static double getDistanceToTarget(Pose2d pose, Pose2d target) {
    return pose.relativeTo(target).getTranslation().getNorm();
  }
}

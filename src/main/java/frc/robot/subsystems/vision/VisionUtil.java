// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.ObservationType;
import frc.robot.util.FieldConstants;

/** Add your docs here. */
public class VisionUtil {
    public static Matrix<N3, N1> getStdDevs(ObservationType type) {
        switch (type) {
        case PhotonPnP:
            return VisionConstants.StandardDevs.multiTagDevs;
        case PhotonTrig:
            return VisionConstants.StandardDevs.trigDevs;
        case LimeLightMegatag2:
            return VisionConstants.StandardDevs.trigDevs;
        case LimeLightMegatag1:
            return VisionConstants.StandardDevs.multiTagDevs;
        default:
            return VisionConstants.StandardDevs.multiTagDevs;
        }
    }

  public static boolean inFieldBounds(Pose2d pose) {
    return pose.getX() >= 0
        && pose.getX() <= FieldConstants.defaultAprilTagType.getFieldLength()
        && pose.getY() >= 0
        && pose.getY() <= FieldConstants.defaultAprilTagType.getFieldWidth();
  }

  public static boolean checkForInverseRead(Pose2d pose, int[] usedIds) {
    Rotation2d poseRotation = pose.getRotation();
    boolean isBad = false;
    for (int i=0; i<usedIds.length; i++) {
        if (usedIds[i] > 0) {
            Pose2d tagPose = FieldConstants.defaultAprilTagType.getTagPose(i).get().toPose2d();
            if (MathUtil.isNear(tagPose.getRotation().getRadians(),poseRotation.getRadians(),Math.PI)) {
                isBad = true;
            }
        }
    }
    return isBad;
  }
}
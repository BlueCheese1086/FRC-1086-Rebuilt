// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldConstants {
    public static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // Use This when Field Gets Release

    public static class AllianceZones {
        public static final double RedToNeutralX = 1.0; // Replace these with the actual values
        public static final double NeutralToBlueX = 2.0; // Replace these with the actual values
    }

    public static boolean inFieldBounds(Pose2d pose) {
        return pose.getX() >= 0 && pose.getX() <= FieldConstants.layout.getFieldLength() && pose.getY() >= 0 && pose.getY() <= FieldConstants.layout.getFieldWidth();
    }

    public static boolean inAllianceZone(Pose2d pose) {
        boolean red = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        if (red) {
            return pose.getX() <= AllianceZones.RedToNeutralX;
        } else {
            return pose.getX() >= AllianceZones.NeutralToBlueX;
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldConstants {
    public static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // Use This when Field Gets Release

    public static final double fieldLength = layout.getFieldLength();
    public static final double fieldWidth = layout.getFieldWidth();

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

    public static class HubConstants {
        public static final Pose2d hubPoseBlue = new Pose2d(new Translation2d(Units.inchesToMeters(158.6+(47/2)), FieldConstants.layout.getFieldWidth()/2),Rotation2d.kZero);
        public static final Pose2d hubPose = AllianceFlipUtil.apply(hubPoseBlue);
    }

    public static class NeutralZone {
        public static final double NeutralZoneStart = Units.inchesToMeters(158.6+47);
        public static final double NeutralZoneEnd = Units.inchesToMeters(fieldLength-NeutralZoneStart);

        public static boolean inZone(Pose2d pose) {
            return (pose.getX() < NeutralZoneEnd) && (pose.getX() > NeutralZoneStart);
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.LimelightSettings.StreamMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {
  private final Limelight limelight;
  private final LimelightPoseEstimator poseEstimator;

  public VisionIOLimelight(String name) {
    limelight = new Limelight("limelight-" + name);
    poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    limelight.getSettings().withImuMode(ImuMode.ExternalImu);
    limelight.getSettings().withLimelightLEDMode(LEDMode.ForceOff);
    limelight.getSettings().withStreamMode(StreamMode.PictureInPictureMain);
  }

  @Override
  public Pose3d getPose() {
    Optional<LimelightResults> results = limelight.getLatestResults();
    if (results.isPresent()) {
      double[] poses = limelight.getLatestResults().get().botpose;
      return new Pose3d(poses[0], poses[1], poses[2], new Rotation3d(poses[3], poses[4], poses[5]));
    } else {
      return Pose3d.kZero;
    }
  }

  @Override
  public void updatePose(Pose2d robotPose) {
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                new Rotation3d(robotPose.getRotation()),
                RadiansPerSecond.zero(),
                RadiansPerSecond.zero(),
                RadiansPerSecond.zero()));
  } // Use this to set sim pose in Sim and update rotation in Real for both

  // PhotonVision and Limelight

  @Override
  public void updateInputs(VisionInputs inputs) {
    Optional<PoseEstimate> poseEstimate = poseEstimator.getPoseEstimate();
    if (poseEstimate.isPresent()) {
      PoseEstimate estimatedPose = poseEstimate.get();
      inputs.estimatedPose = estimatedPose.pose;
      inputs.pose = estimatedPose.pose.toPose2d();
      inputs.type =
          estimatedPose.isMegaTag2
              ? ObservationType.LimeLightMegatag2
              : ObservationType.LimeLightMegatag1;
      inputs.timestamp = estimatedPose.timestampSeconds;
      inputs.averageDistance = estimatedPose.avgTagDist;
      inputs.tagCount = estimatedPose.rawFiducials.length;
      int[] usedTags = new int[estimatedPose.rawFiducials.length];
      inputs.usedTagPoses = new Pose3d[estimatedPose.rawFiducials.length];
      for (int i = 0; i < usedTags.length; i++) {
        usedTags[i] = estimatedPose.rawFiducials[i].id;
        inputs.usedTagPoses[i] =
            VisionConstants.PhysicalConstants.fieldLayout.getTagPose(i).orElse(Pose3d.kZero);
      }
      inputs.tagsUsed = usedTags;
    }
  }
}

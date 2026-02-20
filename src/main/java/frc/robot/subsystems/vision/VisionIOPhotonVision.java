// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  private Pose3d latestPose = new Pose3d();
  private Pose2d currentPose;

  public VisionIOPhotonVision(String name, Transform3d transform) {
    camera = new PhotonCamera(name);
    poseEstimator =
        new PhotonPoseEstimator(VisionConstants.PhysicalConstants.fieldLayout, transform);
  }

  @Override
  public Pose3d getPose() {
    return latestPose;
  }

  @Override
  public void updatePose(Pose2d robotPose) {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), robotPose.getRotation());
    currentPose = robotPose;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      result = results.get(0);
      inputs.type =
          result.getMultiTagResult().isPresent()
              ? ObservationType.PhotonMultiTag
              : (VisionConstants.Strategies.secondary == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
                  ? ObservationType.PhotonTrig
                  : ObservationType.PhotonPnP);
      Optional<EstimatedRobotPose> vPoseEstimated =
          inputs.type == ObservationType.PhotonMultiTag
              ? poseEstimator.estimateCoprocMultiTagPose(result)
              : poseEstimator.estimatePnpDistanceTrigSolvePose(result);
      vPoseEstimated.ifPresent(
          (poseEstimated) -> {
            if (VisionUtil.inFieldBounds(poseEstimated.estimatedPose.toPose2d())
                && MathUtil.applyDeadband(poseEstimated.estimatedPose.getZ(), 0.1) == 0.0) {
              latestPose = poseEstimated.estimatedPose;
              inputs.estimatedPose = poseEstimated.estimatedPose;
              inputs.pose =
                  inputs.type == ObservationType.PhotonMultiTag
                      ? poseEstimated.estimatedPose.toPose2d()
                      : new Pose2d(
                          poseEstimated.estimatedPose.toPose2d().getTranslation(),
                          currentPose.getRotation());
              inputs.tagCount = poseEstimated.targetsUsed.size();
              Pose3d[] tagPoses = new Pose3d[inputs.tagCount];
              int[] tagsUsed = new int[inputs.tagCount];
              for (int i = 0; i < inputs.tagCount; i++) {
                tagsUsed[i] = poseEstimated.targetsUsed.get(i).getFiducialId();
                tagPoses[i] = VisionConstants.PhysicalConstants.fieldLayout.getTagPose(tagsUsed[i]).orElse(Pose3d.kZero);
              }
              inputs.usedTagPoses = tagPoses;
              inputs.tagsUsed = tagsUsed;
              inputs.timestamp = poseEstimated.timestampSeconds;
              inputs.averageDistance = getAverageDist(result);
            }
          });
    }
  }

  private double getAverageDist(PhotonPipelineResult result) {
    if (result.hasTargets()) {
      double dist = result.getTargets().get(0).getBestCameraToTarget().getTranslation().getNorm();
      for (int i = 1; i < result.getTargets().size(); i++) {
        dist += result.getTargets().get(i).getBestCameraToTarget().getTranslation().getNorm();
        dist /= 2;
      }
      return dist;
    } else {
      return 0.0;
    }
  }
}

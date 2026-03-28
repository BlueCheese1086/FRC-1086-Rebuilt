// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.BaseStream;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOPhotonFused implements VisionIO {
  private final PhotonPoseEstimator poseEstimator;
  protected final PhotonCamera camera;
  private final String name;
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOPhotonFused(String name, Transform3d transform, Supplier<Pose2d> poseSupplier) {
    poseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout, transform);
    camera = new PhotonCamera(name);
    this.name = name;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = name;
    inputs.connected = camera.isConnected();
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), poseSupplier.get().getRotation());
    Pose2d refrencePose = poseSupplier.get();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    ArrayList<PoseObservation> observations = new ArrayList<PoseObservation>();
    Logger.recordOutput("Vision/" + name + "/Results", results.size());
    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> estimation = poseEstimator.estimateCoprocMultiTagPose(result);
      Optional<EstimatedRobotPose> trig = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
      Optional<EstimatedRobotPose> pnpLowestAmb = poseEstimator.estimateLowestAmbiguityPose(result);
      Optional<EstimatedRobotPose> pnpBestTarget =
          poseEstimator.estimateAverageBestTargetsPose(result);
      Optional<EstimatedRobotPose> closestToReference =
          poseEstimator.estimateClosestToReferencePose(result, new Pose3d(refrencePose));
      Logger.recordOutput("Vision/" + name + "/Trig", trig.isPresent());
      Logger.recordOutput("Vision/" + name + "/Lowest Ambiguity", pnpLowestAmb.isPresent());
      Logger.recordOutput("Vision/" + name + "/Best Target", pnpBestTarget.isPresent());
      Logger.recordOutput(
          "Vision/" + name + "/Closest to Reference", closestToReference.isPresent());
      Logger.recordOutput("Vision/" + name + "/Multi Tag", estimation.isPresent());
      if (result.multitagResult.isPresent() && estimation.isPresent()) {
        EstimatedRobotPose estimated = estimation.get();
        Pose3d pose = estimated.estimatedPose;
        double dist =
            VisionConstants.fieldLayout
                .getTagPose(estimated.targetsUsed.get(0).fiducialId)
                .get()
                .relativeTo(pose)
                .getTranslation()
                .getNorm();
        for (int i = 1; i < estimated.targetsUsed.size(); i++) {
          dist +=
              VisionConstants.fieldLayout
                  .getTagPose(estimated.targetsUsed.get(i).fiducialId)
                  .get()
                  .relativeTo(pose)
                  .getTranslation()
                  .getNorm();
          dist /= 2;
        }
        Logger.recordOutput(
            "Vision/" + name + "/Pose Estimation/Multitag", estimated.estimatedPose);
        observations.add(
            new PoseObservation(
                estimated.timestampSeconds,
                estimated.estimatedPose,
                result.multitagResult.get().estimatedPose.ambiguity,
                estimated.targetsUsed.size(),
                dist,
                PoseObservationType.PHOTONVISION_MULTITAG));
      } else {
        if (trig.isPresent()
            && pnpLowestAmb.isPresent()
            && pnpBestTarget.isPresent()
            && closestToReference.isPresent()) {
          System.out.println("Got Fused Single Tag");
          Pose3d trigPose = trig.get().estimatedPose;
          Pose3d lowestAmb = pnpLowestAmb.get().estimatedPose;
          Pose3d bestTarget = pnpBestTarget.get().estimatedPose;
          Pose3d closestReference = closestToReference.get().estimatedPose;
          Logger.recordOutput("Vision/" + name + "/Pose Estimation/Trig", trigPose);
          Logger.recordOutput(
              "Vision/" + name + "/Pose Estimation/Lowest Ambiguity", lowestAmb);
          Logger.recordOutput(
              "Vision/" + name + "/Pose Estimation/Best Target", bestTarget);
          Logger.recordOutput(
              "Vision/" + name + "/Pose Estimation/closest", closestReference);
          ArrayList<Pose2d> acceptedPoses = new ArrayList<Pose2d>();
          Pose2d[] poses = new Pose2d[] {trigPose.toPose2d(),lowestAmb.toPose2d(),bestTarget.toPose2d(),closestReference.toPose2d()};
          for (int i=0; i<poses.length; i++) {
            if (poses[i].getX() < 0) {}
          }
          
        } else if (trig.isPresent()) {
          System.out.println("Got Trignometry");
          Pose3d trigPose = trig.get().estimatedPose;
          Logger.recordOutput("Vision/" + name + "/Pose Estimation/Trig", trigPose);
        } else if (pnpLowestAmb.isPresent()) {
          System.out.println("Got Lowest Ambiguity");
          Pose3d lowestAmb = trig.get().estimatedPose;
          Logger.recordOutput(
              "Vision/" + name + "/Pose Estimation/Lowest Ambiguity", lowestAmb);
        }
      }
    }
    inputs.poseObservations = observations.toArray(new PoseObservation[observations.size()]);
  }
}

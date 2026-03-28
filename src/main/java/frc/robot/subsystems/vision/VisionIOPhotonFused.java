// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOPhotonFused implements VisionIO {
  private final PhotonPoseEstimator poseEstimator;
  protected final PhotonCamera camera;
  private final String name;

  public VisionIOPhotonFused(String name, Transform3d transform, Rotation2d rotationSupplier) {
    poseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout, transform);
    camera = new PhotonCamera(name);
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = name;
    inputs.connected = camera.isConnected();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
  }
}

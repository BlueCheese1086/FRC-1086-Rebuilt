// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class VisionIOSim extends VisionIOPhotonVision {
  private final PhotonCameraSim sim;
  private static VisionSystemSim enviorment;

  public VisionIOSim(String name, Transform3d transform3d) {
    super(name, transform3d);
    if (enviorment == null) {
      enviorment = new VisionSystemSim("photonSim");
    }
    sim =
        new PhotonCameraSim(
            camera,
            SimCameraProperties.PERFECT_90DEG(),
            VisionConstants.PhysicalConstants.fieldLayout);
    enviorment.addCamera(sim, transform3d);
    enviorment.addAprilTags(VisionConstants.PhysicalConstants.fieldLayout);
  }

  @Override
  public void updatePose(Pose2d pose) {
    enviorment.update(pose);
  }
}

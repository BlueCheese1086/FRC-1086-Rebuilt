// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import limelight.Limelight;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {
    private final Limelight limelight;

    public VisionIOLimelight(String name) {
        limelight = new Limelight("limelight-"+name);
    }

  @Override
  public Pose3d getPose() {
    return Pose3d.kZero;
  }

  @Override
  public void updatePose(
      Pose2d robotPose) {} // Use this to set sim pose in Sim and update rotation in Real for both

  // PhotonVision and Limelight

  @Override
  public void updateInputs(VisionInputs inputs) {}
}

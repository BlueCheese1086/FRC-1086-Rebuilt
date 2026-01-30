// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public class VisionInputs {
    public Pose2d pose = Pose2d.kZero;
    public Pose3d estimatedPose = Pose3d.kZero;
    public double timestamp = 0.0;
    public int tagCount = 0;
    public double averageDistance = 0.0;
    public ObservationType type = ObservationType.Empty;
  }

  public static enum ObservationType {
    PhotonMultiTag,
    PhotonTrig,
    PhotonPnP,
    LimeLightMegatag1,
    LimeLightMegatag2,
    Empty
  }

  public default Pose3d getPose() {
    return Pose3d.kZero;
  }

  public default void updatePose(
      Pose2d robotPose) {} // Use this to set sim pose in Sim and update rotation in Real for both
  // PhotonVision and Limelight

  public default void updateInputs(VisionInputs inputs) {}
}

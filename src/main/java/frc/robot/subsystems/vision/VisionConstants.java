// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */
public class VisionConstants {
  public static class PhysicalConstants {
    public static final Transform3d[] cameraTransforms =
        new Transform3d[] { // Camera Transforms Left to Right,
          // Front to Back
          new Transform3d(
              Units.inchesToMeters(12.066),
              Units.inchesToMeters(11.906),
              Units.inchesToMeters(8.355),
              new Rotation3d(
                  0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(-35))), // Front
          // Left
          // Camera
          new Transform3d(
              Units.inchesToMeters(12.066),
              Units.inchesToMeters(-11.906),
              Units.inchesToMeters(8.355),
              new Rotation3d(
                  0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(35))) // Front
          // Right
          // Camera
        };

    public static final AprilTagFieldLayout fieldLayout = FieldConstants.defaultAprilTagType;
  }

  public static class Strategies {
    public static final PoseStrategy primary = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy secondary = PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;
  }

  public static class StandardDevs {
    public static final Matrix<N3, N1> multiTagDevs = VecBuilder.fill(1, 1, 1);
    public static final Matrix<N3, N1> trigDevs = VecBuilder.fill(1, 1, Double.MAX_VALUE);
  }
}

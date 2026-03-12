package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.25; // 0.3d
  public static double maxZError = 0.19; // 0.75

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
  //   public static AprilTagFieldLayout fieldLayout =
  //       AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  static {
    try {
      fieldLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("fields/1086_Regency_Field3-7.json"));
    } catch (IOException e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }
  }

  public static Transform3d robotToLeftCam =
      new Transform3d(
          -Units.inchesToMeters(12.5),
          -Units.inchesToMeters(12),
          Units.inchesToMeters(9.85),
          new Rotation3d(0.0, Units.degreesToRadians(165.0), Units.degreesToRadians(0)));
  public static Transform3d robotToRightCam =
      new Transform3d(
          -Units.inchesToMeters(12.5),
          Units.inchesToMeters(12),
          Units.inchesToMeters(9.85),
          new Rotation3d(0.0, Units.degreesToRadians(165.0), Units.degreesToRadians(0)));

  public static double trigLinearStdDevBaseline = 0.381; // Meters
  public static double trigAngularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  public static double multitagLinearStdDevBaseline = 0.6858; // Meters
  public static double multitagAngularStdDevBaseline = 0.03; // Radians

  public static double averageTagDistance = 2.28;
}

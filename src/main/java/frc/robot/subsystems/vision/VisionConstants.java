package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3; // 0.3d
  public static double maxZError = 1.245; // 0.75

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        0.8
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.45; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  // public static AprilTagFieldLayout fieldLayout;
  //   AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  // hi
  public static Transform3d robotToLeftCam =
      new Transform3d(
          -Units.inchesToMeters(11.073859),
          Units.inchesToMeters(8.995264),
          Units.inchesToMeters(11.348081),
          new Rotation3d(
              Units.degreesToRadians(7.6307485467),
              -Units.degreesToRadians(30.028483379),
              Units.degreesToRadians(180.0 + 17.1270263437)));
  public static Transform3d robotToRightCam =
      new Transform3d(
          -Units.inchesToMeters(11.073859),
          -Units.inchesToMeters(8.995264),
          Units.inchesToMeters(11.348081),
          new Rotation3d(
              -Units.degreesToRadians(7.6307485467),
              -Units.degreesToRadians(29.9999866973),
              Units.degreesToRadians(180.0 - 17.1921978943)));

  public static double trigLinearStdDevBaseline = 2.55;
  public static double trigAngularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  public static double multitagLinearStdDevBaseline = 1.5; // Meters
  public static double multitagAngularStdDevBaseline = 0.03; // Radians

  public static double averageTagDistance = 4.5;
}

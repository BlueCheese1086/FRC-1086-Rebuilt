package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

/** Add your docs here. */
public class VisionConstants {

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.4;
  public static double maxZError = 1.245; // 0.75

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0
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
          -Units.inchesToMeters(11.01875),
          Units.inchesToMeters(8.5),
          Units.inchesToMeters(10.75),
          new Rotation3d(
              -Units.degreesToRadians(7.6307485467),
              -Units.degreesToRadians(20.0),
              Units.degreesToRadians(180.0 + 17.1270263437)));
  public static Transform3d robotToRightCam =
      new Transform3d(
          -Units.inchesToMeters(11.01875),
          -Units.inchesToMeters(8.25),
          Units.inchesToMeters(10.75),
          new Rotation3d(
              Units.degreesToRadians(7.6307485467),
              -Units.degreesToRadians(20.0),
              Units.degreesToRadians(180.0 - 17.1921978943)));

  public static double trigLinearStdDevBaseline = 2.55;
  public static double trigAngularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  public static double multitagLinearStdDevBaseline = 0.9; // Meters
  public static double multitagAngularStdDevBaseline = 0.03; // Radians

  public static double averageTagDistance = Double.POSITIVE_INFINITY;
  public static double averageTagDistanceSingleTag = 7.5;
  public static double maxFusedDistance = 0.125;
  public static PoseObservationType preferred = PoseObservationType.PHOTONVISION_TRIG;
}

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class VisionConstants {

  public static LoggedTunableNumber tunableMultiTagLinearBaseline =
      new LoggedTunableNumber("/Vision/Tuning/MultiTagLinearBaseline", 0.8);
  public static LoggedTunableNumber tunableMultiTagAngularBaseline =
      new LoggedTunableNumber("/Vision/Tuning/MultiTagAngluarBaseline", 0.03);
  public static LoggedTunableNumber tunableTrigLinearBaseline =
      new LoggedTunableNumber("/Vision/Tuning/TrigLinearBaseline", 0.5);

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 1.245; // 0.75

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0 // Limelight
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.32; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static double linearStdDevMegatag1Factor = 0.8;
  public static double angularStdDevMegatag1Factor = 0.35;
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
              -Units.degreesToRadians(30.0),
              Units.degreesToRadians(180.0 + 17.1270263437)));
  public static Transform3d robotToRightCam =
      new Transform3d(
          -Units.inchesToMeters(11.01875),
          -Units.inchesToMeters(8.25),
          Units.inchesToMeters(10.75),
          new Rotation3d(
              Units.degreesToRadians(7.6307485467),
              -Units.degreesToRadians(30.0),
              Units.degreesToRadians(180.0 - 17.1921978943)));

  public static double trigLinearStdDevBaseline = 0.5;
  public static double trigAngularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  public static double multitagLinearStdDevBaseline = 0.08;
  public static double multitagAngularStdDevBaseline = 0.03;

  public static double averageTagDistance = Double.POSITIVE_INFINITY;
  public static double averageTagDistanceSingleTag = 7.5;
  public static double averageMultitag2SingleTagDistance = 3.75;
  public static double maxFusedDistance = 0.125;
  public static PoseObservationType preferred = PoseObservationType.PHOTONVISION_TRIG;
}

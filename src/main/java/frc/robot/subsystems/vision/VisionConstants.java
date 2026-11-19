package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;

public class VisionConstants {
  // Keep static and tunable defaults consistent with the supplied tunable value.
  public static LoggedTunableNumber tunableMultiTagLinearBaseline =
      new LoggedTunableNumber("/Vision/Tuning/MultiTagLinearBaseline", 0.8);
  public static LoggedTunableNumber tunableMultiTagAngularBaseline =
      new LoggedTunableNumber("/Vision/Tuning/MultiTagAngluarBaseline", 0.03);
  public static LoggedTunableNumber tunableTrigLinearBaseline =
      new LoggedTunableNumber("/Vision/Tuning/TrigLinearBaseline", 0.5);

  public static double maxAmbiguity = 0.3;
  // This is estimated ROBOT Z, not the camera mounting height. Allows modest bump motion.
  public static double maxZError = 0.35;

  // Initial filtering values, to validate against actual camera data.
  public static final double maxVisionAgeSeconds = 0.50;
  public static final double maxVisionFutureSeconds = 0.02;
  public static final double maxVisionTranslationInnovationMeters = 0.75;
  // Use vision translation and gyro heading while diagnosing the oscillation.
  public static final boolean useVisionHeading = false;

  public static double[] cameraStdDevFactors = {1.0, 1.0, 1.0};
  public static double linearStdDevMegatag2Factor = 0.32;
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
  public static double linearStdDevMegatag1Factor = 0.8;
  public static double angularStdDevMegatag1Factor = 0.35;

  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static Transform3d robotToLeftCam =
      new Transform3d(
          -Units.inchesToMeters(11.01875),
          Units.inchesToMeters(8.5),
          Units.inchesToMeters(10.75),
          new Rotation3d(
              -Units.degreesToRadians(7.6307485467),
              -Units.degreesToRadians(30.0),
              Units.degreesToRadians(180.0 + 17.1270263437)));

  public static Transform3d robotToCenter =
      new Transform3d(
              new Translation3d(0, 0, Units.inchesToMeters(21.143719 + 0.0625 + 3.125)),
              new Rotation3d(Degrees.zero(), Degrees.of(-18.1), Degrees.zero()))
          .plus(
              new Transform3d(
                  new Translation3d(24.1 / 1000.0, 0.0, Units.inchesToMeters(-0.3125)),
                  Rotation3d.kZero));

  public static double trigLinearStdDevBaseline = 0.5;
  public static double trigAngularStdDevBaseline = Double.POSITIVE_INFINITY;
  public static double multitagLinearStdDevBaseline = 0.8;
  public static double multitagAngularStdDevBaseline = 0.03;

  public static double averageTagDistance = 6.0;
  public static double averageTagDistanceSingleTag = 4.5;
  public static double averageMultitag2SingleTagDistance = 4.5;
  public static double maxFusedDistance = 0.125;
  public static PoseObservationType preferred = PoseObservationType.PHOTONVISION_TRIG;
}

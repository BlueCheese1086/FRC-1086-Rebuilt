package frc.robot.subsystems.vision;

/*
 * Limelight integration notes (commented out).
 *
 * This file demonstrates how to integrate the WCP-style Limelight
 * implementation (LimelightWCP.java) into the existing Vision fusion
 * pipeline. The code is intentionally commented so it doesn't compile
 * or affect runtime. Uncomment and adapt after competition.
 *
 * Example usage:
 *
 * // Imports you'll likely need:
 * // import java.util.Optional;
 * // import edu.wpi.first.math.Matrix;
 * // import edu.wpi.first.math.VecBuilder;
 * // import edu.wpi.first.math.geometry.Pose2d;
 * // import edu.wpi.first.math.numbers.N1;
 * // import edu.wpi.first.math.numbers.N3;
 *
 * // Create the Limelight instance (e.g. in Vision constructor):
 * // private final Limelight limelightFront = new Limelight("limelight-front");
 *
 * // When producing measurements for the fusion step (pseudo-code):
 * // Pose2d currentPose = getCurrentOdometryPose();
 * // Optional<Limelight.Measurement> llMeas = limelightFront.getMeasurement(currentPose);
 * // if (llMeas.isPresent()) {
 * //     var m = llMeas.get();
 * //     // m.poseEstimate contains the PoseEstimate returned by LimelightHelpers
 * //     // m.standardDeviations is a Matrix<N3,N1> giving (x,y,theta) std devs
 * //     // Convert and add to your vision fusion (example method names):
 * //     // addVisionMeasurement(m.poseEstimate.pose, m.standardDeviations, m.poseEstimate.timestamp);
 * // }
 *
 * Notes / tips:
 * - The example in LimelightWCP uses a large rotation stddev (10.0) to downweight
 *   the camera's rotation when fusing. Tune these numbers to match your sensors.
 * - Make sure LimelightHelpers referenced by LimelightWCP is available and imported
 *   in the same package or adjust the package path.
 * - This file intentionally contains no active code. Remove the comments
 *   and copy the snippets into your `Vision` class when you're ready.
 */

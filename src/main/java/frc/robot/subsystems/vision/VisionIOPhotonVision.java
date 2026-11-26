package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected Transform3d robotToCamera;
  protected final AprilTagFieldLayout aprilTagLayout;
  private final Supplier<Rotation2d> rotationSupplier;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name             The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.aprilTagLayout = VisionConstants.fieldLayout;
    this.rotationSupplier = rotationSupplier;
    camera.setDriverMode(false);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.cameraName = camera.getName();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation = new TargetObservation(
            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
            Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION_MULTITAG)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d cameraToTarget = target.bestCameraToTarget;

          // Ideally, sample field-relative heading at result.getTimestampSeconds().
          Rotation2d heading = rotationSupplier.get();

          double distance = target.bestCameraToTarget.getTranslation().getNorm();

          // Reconstruct camera-to-tag vector from range and measured image angles.
          // Rotate into robot coordinates, then into field coordinates.
          Translation2d cameraToTagField = new Translation3d(
              distance,
              new Rotation3d(
                  0.0,
                  -Math.toRadians(target.getPitch()),
                  -Math.toRadians(target.getYaw())))
              .rotateBy(robotToCamera.getRotation())
              .toTranslation2d()
              .rotateBy(heading);

          // Tag position minus camera-to-tag vector gives camera position.
          Translation2d cameraPosition = tagPose.get().getTranslation().toTranslation2d()
              .minus(cameraToTagField);

          // Account for the camera's offset from the robot center.
          Translation2d robotPosition = cameraPosition.minus(
              robotToCamera.getTranslation().toTranslation2d()
                  .rotateBy(heading));

          Pose3d robot = new Pose3d(new Pose2d(robotPosition, heading));

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robot, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION_TRIG)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  public void setTransforms(Transform3d transform) {
    robotToCamera = transform;
  }
}

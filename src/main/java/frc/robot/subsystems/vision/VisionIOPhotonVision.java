package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final AprilTagFieldLayout aprilTagLayout;
  private final PhotonPoseEstimator poseEstimator;
  private final Supplier<Rotation2d> rotationSupplier;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String name,
      Transform3d robotToCamera,
      Supplier<Rotation2d> rotationSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.aprilTagLayout = VisionConstants.fieldLayout;
    this.rotationSupplier = rotationSupplier;
    this.poseEstimator =
        new PhotonPoseEstimator(VisionConstants.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    camera.setDriverMode(false);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      // Update latest target observation
      poseEstimator.addHeadingData(result.getTimestampSeconds(), rotationSupplier.get());
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (result.hasTargets()) {
        poseEstimator
            .update(result)
            .ifPresent(
                (robotposeEstiamted) -> {
                  boolean isMultiTag =
                      robotposeEstiamted.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
                  if (isMultiTag) {
                    double totalTagDis = 0.0;
                    for (PhotonTrackedTarget target : robotposeEstiamted.targetsUsed) {
                      totalTagDis += target.bestCameraToTarget.getTranslation().getNorm();
                    }

                    tagIds.addAll(
                        robotposeEstiamted.targetsUsed.stream()
                            .map(t -> (short) t.fiducialId)
                            .toList());

                    poseObservations.add(
                        new PoseObservation(
                            robotposeEstiamted.timestampSeconds, // Timestamp
                            robotposeEstiamted.estimatedPose, // 3D pose estimate
                            result.multitagResult.get().estimatedPose.ambiguity, // Ambiguity
                            robotposeEstiamted.targetsUsed.size(), // Tag count
                            totalTagDis
                                / robotposeEstiamted.targetsUsed.size(), // Average tag distance
                            PoseObservationType.PHOTONVISION)); // Observation type

                  } else {
                    PhotonTrackedTarget target = robotposeEstiamted.targetsUsed.get(0);

                    tagIds.add((short) robotposeEstiamted.targetsUsed.get(0).fiducialId);

                    poseObservations.add(
                        new PoseObservation(
                            robotposeEstiamted.timestampSeconds, // Timestamp
                            robotposeEstiamted.estimatedPose, // 3D pose estimate
                            target.poseAmbiguity, // Ambiguity
                            1, // Tag count
                            target
                                .getBestCameraToTarget()
                                .getTranslation()
                                .getNorm(), // Average tag distance
                            PoseObservationType.PHOTONVISION)); // Observation type
                  }
                });
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
}
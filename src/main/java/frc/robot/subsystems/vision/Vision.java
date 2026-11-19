// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.rejectionReason;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + inputs[i].cameraName, inputs[i]);
    }

    if (VisionConstants.tunableMultiTagLinearBaseline.hasChanged(hashCode())) {
      VisionConstants.multitagLinearStdDevBaseline = tunableMultiTagLinearBaseline.getAsDouble();
    }

    if (VisionConstants.tunableMultiTagAngularBaseline.hasChanged(hashCode())) {
      VisionConstants.multitagAngularStdDevBaseline = tunableMultiTagAngularBaseline.getAsDouble();
    }

    if (VisionConstants.tunableTrigLinearBaseline.hasChanged(hashCode())) {
      VisionConstants.trigLinearStdDevBaseline = tunableTrigLinearBaseline.getAsDouble();
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // System.out.println(inputs[cameraIndex].cameraName);

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = VisionConstants.fieldLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }
      rejectionReason[] rejections =
          new rejectionReason[inputs[cameraIndex].poseObservations.length];
      int rejectionId = 0;
      // Loop over pose observations
      for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
        double now = Timer.getFPGATimestamp();
        boolean noTags = observation.tagCount() <= 0;
        boolean badAmbiguity =
            observation.tagCount() == 1
                && (observation.ambiguity() > maxAmbiguity
                    || (!Double.isFinite(observation.ambiguity())
                        && observation.type() != PoseObservationType.PHOTONVISION_TRIG));
        boolean badPose =
            !Double.isFinite(observation.pose().getX())
                || !Double.isFinite(observation.pose().getY())
                || !Double.isFinite(observation.pose().getZ())
                || !Double.isFinite(observation.pose().getRotation().getX())
                || !Double.isFinite(observation.pose().getRotation().getY())
                || !Double.isFinite(observation.pose().getRotation().getZ());
        boolean badHeight = badPose || Math.abs(observation.pose().getZ()) > maxZError;
        boolean badDistance =
            !Double.isFinite(observation.averageTagDistance())
                || observation.averageTagDistance() <= 0
                || observation.averageTagDistance() >= averageTagDistance;
        boolean limelight =
            observation.type() == PoseObservationType.LIMELIGHT_MEGATAG_2
                || observation.type() == PoseObservationType.LIMELIGHT_MEGATAG_1;
        boolean distantSingleTag =
            observation.tagCount() == 1
                && observation.averageTagDistance()
                    >= (limelight
                        ? averageMultitag2SingleTagDistance
                        : averageTagDistanceSingleTag);
        boolean badTimestamp =
            !Double.isFinite(observation.timestamp())
                || now - observation.timestamp() > maxVisionAgeSeconds
                || observation.timestamp() - now > maxVisionFutureSeconds;
        boolean beyondX = observation.pose().getX() >= fieldLayout.getFieldLength();
        boolean beyondY = observation.pose().getY() >= fieldLayout.getFieldWidth();
        boolean belowX = observation.pose().getX() <= 0.0;
        boolean belowY = observation.pose().getY() <= 0.0;
        boolean rejectPose =
            noTags
                || badAmbiguity
                || badHeight
                || badDistance
                || distantSingleTag
                || badTimestamp
                || beyondX
                || beyondY
                || belowX
                || belowY
                || !inputs[cameraIndex].connected;
        rejections[rejectionId] =
            new rejectionReason(
                noTags,
                badAmbiguity,
                badHeight,
                badDistance,
                distantSingleTag,
                beyondX,
                beyondY,
                belowX,
                belowY,
                badTimestamp,
                observation.type(),
                observation.pose());

        rejectionId++;
        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) robotPosesRejected.add(observation.pose());

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev =
            (observation.type() == PoseObservationType.PHOTONVISION_TRIG
                    ? trigLinearStdDevBaseline
                    : multitagLinearStdDevBaseline)
                * stdDevFactor;
        double angularStdDev =
            (observation.type() == PoseObservationType.PHOTONVISION_TRIG
                    ? trigAngularStdDevBaseline
                    : multitagAngularStdDevBaseline)
                * stdDevFactor;
        if (observation.type() == PoseObservationType.LIMELIGHT_MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        if (observation.type() == PoseObservationType.LIMELIGHT_MEGATAG_1) {
          linearStdDev *= linearStdDevMegatag1Factor;
          angularStdDev *= angularStdDevMegatag1Factor;
        }

        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (!Double.isFinite(linearStdDev)
            || linearStdDev <= 0
            || Double.isNaN(angularStdDev)
            || angularStdDev <= 0) {
          robotPosesRejected.add(observation.pose());
          continue;
        }
        // This list means quality-filtered candidates. Drive logs the final fusion decision.
        robotPosesAccepted.add(observation.pose());

        // allVisionObservations.add(
        //     new VisionObservation(
        //         observation.timestamp(),
        //         observation.pose(),
        //         VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
      Logger.recordOutput(
          "Vision/Camera " + inputs[cameraIndex].cameraName + "/Rejections", rejections);
      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + inputs[cameraIndex].cameraName + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + inputs[cameraIndex].cameraName + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + inputs[cameraIndex].cameraName + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + inputs[cameraIndex].cameraName + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // if (!DriverStation.isAutonomousEnabled() || DriverStation.isDisabled()) {
    //   allVisionObservations.stream()
    //       .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
    //       .forEach(RobotState.getInstance()::addVisionObservation);
    // }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

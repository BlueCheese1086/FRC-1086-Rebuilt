// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseMath;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  VisionIO[] cameras;
  VisionConsumer consumer;
  Supplier<Pose2d> poseSupplier;
  Pose2d[] poses;
  Pose3d[] tagPoses;
  VisionInputsAutoLogged[] inputs;
  public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier,VisionIO... cameras) {
    this.cameras = cameras;
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    poses = new Pose2d[cameras.length];
    inputs = new VisionInputsAutoLogged[cameras.length];
    for (int i=0;i<inputs.length;i++) {
      inputs[i] = new VisionInputsAutoLogged();
    }
    tagPoses = new Pose3d[FieldConstants.defaultAprilTagType.getTags().size()];
    for (int i=0;i<tagPoses.length; i++) {
      tagPoses[i] = FieldConstants.defaultAprilTagType.getTags().get(i).pose;
    }
    Logger.recordOutput("Vision/Tags", tagPoses);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Vision/Camera Transforms", VisionConstants.PhysicalConstants.cameraTransforms);
    // This method will be called once per scheduler run
    for (int i=0; i<cameras.length; i++) {
      poses[i] = cameras[i].getPose().toPose2d();
      cameras[i].updateInputs(inputs[i]);
      cameras[i].updatePose(poseSupplier.get());
      Logger.processInputs("Vision/Camera "+i,inputs[i]);
      if (Robot.isReal()) {
        if (VisionConstants.inFieldBounds(inputs[i].pose) && (MathUtil.applyDeadband(Timer.getFPGATimestamp()-inputs[i].timestamp,2.5) == 0.0)) {
          consumer.accept(inputs[i].pose, inputs[i].timestamp, calculateSTDDevs(inputs[i]));
        }
      }
    }
    if (Robot.isReal()) {
      // Pose2d average = PoseMath.average(poses);
      // if (FieldConstants.inFieldBounds(average)) {
      //   consumer.accept(average, Timer.getFPGATimestamp(), VecBuilder.fill(3, 3, 3));
      // }
    } else {
      Logger.recordOutput("Vision/Estimated Pose", PoseMath.average(poses));
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private Matrix<N3,N1> calculateSTDDevs(VisionInputs inputs) {
    Matrix<N3,N1> stdDevs = VisionConstants.StandardDevs.getStdDevs(inputs.type);
    return stdDevs;
  }
}
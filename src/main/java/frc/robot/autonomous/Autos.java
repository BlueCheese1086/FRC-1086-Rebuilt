// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {

  private static final PIDController xControl =
      new PIDController(
          Preferences.getDouble("Autos_X_P", 8.0), 0, Preferences.getDouble("Autos_X_D", 0.0));
  private static final PIDController yControl =
      new PIDController(
          Preferences.getDouble("Autos_Y_P", 8.0), 0, Preferences.getDouble("Autos_Y_D", 0.0));
  private static final PIDController rotControl =
      new PIDController(
          Preferences.getDouble("Autos_Rot_P", 8.0), 0, Preferences.getDouble("Autos_Rot_D", 0.0));

  static {
    rotControl.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static AutoFactory factory;
  private static Drive drive;
  private static Intake intake;

  private static LoggedTunableNumber runSubsystems = new LoggedTunableNumber("Autos/Run Systems");

  public static void setup(Drive kDrive, Intake kIntake) {
    runSubsystems = new LoggedTunableNumber("Autos/Run Systems", 0);
    drive = kDrive;
    intake = kIntake;
    factory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            run(),
            DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
            drive);
  }

  public static Command runAutonomous(String path) {
    AutoRoutine routine = factory.newRoutine("auto");
    AutoTrajectory traj = routine.trajectory(path);

    traj.atTime("IntakeDown").onTrue(intakeDown().onlyIf(() -> (runSubsystems.get() == 1)));
    traj.atTime("IntakeUp").onTrue(intakeUp().onlyIf(() -> (runSubsystems.get() == 1)));
    traj.atTime("IntakeRun")
        .onTrue(
            intakeRun()
                .onlyIf(() -> (runSubsystems.get() == 1))
                .until(traj.atTime("IntakeStop")::getAsBoolean));

    return traj.cmd();
  }

  private static Consumer<SwerveSample> run() {
    return (sample) -> {
      Pose2d targetPose = sample.getPose();
      Pose2d currentPose = drive.getPose();
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.vx + xControl.calculate(currentPose.getX(), targetPose.getX()),
              sample.vy + yControl.calculate(currentPose.getY(), targetPose.getY()),
              sample.omega
                  + rotControl.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians()),
              currentPose.getRotation());
      drive.runVelocity(speeds);
      Logger.recordOutput("Autos/Target Pose", targetPose);
      Logger.recordOutput("Autos/Set Speed", speeds);
    };
  }

  private static Command intakeUp() {
    return intake.setPosition(IntakeConstants.Setpoints.stowed);
  }

  private static Command intakeDown() {
    return intake.setPosition(IntakeConstants.Setpoints.deployed);
  }

  private static Command intakeRun() {
    return intake.setVoltage(IntakeConstants.Setpoints.run);
  }
}

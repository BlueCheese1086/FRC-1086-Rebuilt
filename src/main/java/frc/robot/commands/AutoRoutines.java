// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
// import dev.doglog.DogLog;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Consumer;
// import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoRoutines {

  private static AutoFactory factory;
  private static Drive kDrive;

  public static void setup(Drive drive) {
    kDrive = drive;
    factory = new AutoFactory(drive::getPose, drive::setPose, run(), true, drive);

    NamedCommands.registerCommand("deployIntake", Commands.print("Passed marker!"));
    NamedCommands.registerCommand("startFeeder", Commands.print("Passed marker!"));
  }

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

  private static Consumer<SwerveSample> run() {
    return (sample) -> {
      Pose2d targetPose = sample.getPose();
      Pose2d currentPose = kDrive.getPose();
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.vx + xControl.calculate(currentPose.getX(), targetPose.getX()),
              sample.vy + yControl.calculate(currentPose.getY(), targetPose.getY()),
              sample.omega
                  + rotControl.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians()),
              currentPose.getRotation());
      kDrive.runVelocity(speeds);
      Logger.recordOutput("Autos/Target Pose", targetPose);
      Logger.recordOutput("Autos/Set Speed", speeds);
    };
  }

  public static Command runPath(String pathName, boolean resetPose) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
      return Commands.runOnce(
              () -> {
                Pose2d startPose =
                    path.getStartingHolonomicPose()
                        .orElseGet(() -> new Pose2d(path.getPoint(0).position, new Rotation2d()));
                if (resetPose) {
                  boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                  if (isRed) {
                    startPose = AllianceFlipUtil.apply(startPose);
                  }
                  kDrive.setPose(startPose);
                }

                Logger.recordOutput(
                    "Autos/Selected Path", path.getPathPoses().toArray(new Pose2d[0]));
              })
          .andThen(AutoBuilder.followPath(path))
          .finallyDo(kDrive::stop);

    } catch (Exception e) {
      DriverStation.reportError("Choreo Path Error: " + pathName, e.getStackTrace());
      return Commands.none();
    }
  }

  private static boolean hasWarned = false;

  @SuppressWarnings("unused")
  public static void periodic() {
    if (!DriverStation.isFMSAttached()) {
      boolean updateX = false;
      boolean updateY = false;
      boolean updateRot = false;
    } else {
      if (!hasWarned) {
        DriverStation.reportWarning(
            "FMS Attached so not entering or using tuning mode. Only uses the values currently saved on the robot",
            false);
      }
    }
  }

  private static SysIdRoutine linearRoutine;
  private static Translation2d initialTranslation;
  private static double appliedLinearVelocity = 0.0;

  public static Command autoTranslationSysId(Drive drive) {
    linearRoutine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Second), Volts.of(3), Seconds.of(6)),
            new Mechanism(
                (applied) -> {
                  appliedLinearVelocity = applied.in(Volts);

                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          applied.in(Volts), 0, 0, drive.getRotation()));
                },
                (log) -> {
                  log.motor("AutoTranslate")
                      .voltage(Volts.of(appliedLinearVelocity))
                      .linearPosition(
                          Meters.of(
                              drive.getPose().getTranslation().minus(initialTranslation).getX()))
                      .linearVelocity(
                          MetersPerSecond.of(
                              ChassisSpeeds.fromFieldRelativeSpeeds(
                                      drive.getChassisSpeeds(), drive.getRotation())
                                  .vxMetersPerSecond));
                },
                drive));
    return Commands.sequence(
            linearRoutine.quasistatic(Direction.kForward).withTimeout(6.0),
            linearRoutine.quasistatic(Direction.kReverse).withTimeout(6.0),
            linearRoutine.dynamic(Direction.kForward).withTimeout(6.0),
            linearRoutine.dynamic(Direction.kReverse).withTimeout(6.0))
        .beforeStarting(
            () -> {
              initialTranslation = drive.getPose().getTranslation();
            })
        .finallyDo(drive::stop);
  }

  private static Rotation2d initialRotation;
  private static SysIdRoutine rotationRoutine;
  private static double appliedRotationVelocity = 0.0;

  public static Command autoRotationSysId(Drive drive) {
    rotationRoutine =
        new SysIdRoutine(
            new Config(Volts.of(Math.PI / 3).per(Second), Volts.of(Math.PI * 2), Seconds.of(6.0)),
            new Mechanism(
                (applied) -> {
                  appliedRotationVelocity = applied.in(Volts);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, applied.in(Volts)));
                },
                (log) -> {
                  log.motor("Auto Rotation")
                      .voltage(Volts.of(appliedRotationVelocity))
                      .angularPosition(drive.getRotation().minus(initialRotation).getMeasure())
                      .angularVelocity(
                          RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond));
                },
                drive));
    return Commands.sequence(
            rotationRoutine.quasistatic(Direction.kForward).withTimeout(6.0),
            rotationRoutine.quasistatic(Direction.kReverse).withTimeout(6.0),
            rotationRoutine.dynamic(Direction.kForward).withTimeout(6.0),
            rotationRoutine.dynamic(Direction.kReverse).withTimeout(6.0))
        .beforeStarting(
            () -> {
              initialRotation = drive.getRotation();
            })
        .finallyDo(drive::stop);
  }
}

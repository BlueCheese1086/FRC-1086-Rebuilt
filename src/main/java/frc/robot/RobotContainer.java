// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoBuilder;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOServo;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.FeederIO;
import frc.robot.subsystems.shooter.FeederIOSim;
import frc.robot.subsystems.shooter.FeederIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShootingManager;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  private final Hood hood;
  private final Climb climb;
  private final ShootingManager shootingManager;

  @SuppressWarnings("unused")
  private final Superstructure superstructure;

  private final AutoBuilder autobuilder;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedTunableNumber calibRpm =
      new LoggedTunableNumber("Shooting/Calib/Rpm", 3200.0);
  private final LoggedTunableNumber calibAngleDeg =
      new LoggedTunableNumber("Shooting/Calib/AngleDeg", 65.0);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        shootingManager =
            new ShootingManager(drive::getPose, drive::getChassisSpeeds, drive::getRotation);

        vision =
            new Vision(
                (pose, timestamp, stdDevs) -> {
                  drive.addVisionMeasurement(pose, timestamp, stdDevs);
                  shootingManager.addVisionMeasurement(pose, timestamp);
                },
                drive::getPose,
                // new VisionIOPhotonVision(
                //     "left", VisionConstants.PhysicalConstants.cameraTransforms[0]),
                // new VisionIOPhotonVision(
                //     "right", VisionConstants.PhysicalConstants.cameraTransforms[1]),
                new VisionIOLimelight("scoring"));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter =
            new Shooter(
                new FeederIOTalonFX(RobotMap.ShooterMap.feeder),
                new ShooterIOTalonFX(RobotMap.ShooterMap.left,true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.middle,true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.right, false));
        hood = new Hood(new HoodIOServo());
        climb = new Climb(new ClimbIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        shootingManager =
            new ShootingManager(drive::getPose, drive::getChassisSpeeds, drive::getRotation);

        vision =
            new Vision(
                (pose, timestamp, stdDevs) -> {
                  drive.addVisionMeasurement(pose, timestamp, stdDevs);
                  shootingManager.addVisionMeasurement(pose, timestamp);
                },
                drive::getPose,
                new VisionIOSim("left", VisionConstants.PhysicalConstants.cameraTransforms[0]),
                new VisionIOSim("right", VisionConstants.PhysicalConstants.cameraTransforms[1]));
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter =
            new Shooter(
                new FeederIOSim(), new ShooterIOSim(), new ShooterIOSim(), new ShooterIOSim());
        hood = new Hood(new HoodIOSim());
        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shootingManager = new ShootingManager();

        vision =
            new Vision(
                (pose, timestamp, stdDevs) -> drive.addVisionMeasurement(pose, timestamp, stdDevs),
                drive::getPose,
                new VisionIO() {},
                new VisionIO() {});

        shooter = new Shooter(new FeederIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hood = new Hood(new HoodIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    AutoRoutines.setup(drive);

    Superstructure.ControllerLayout.scoreRequest = driver.rightTrigger();
    Superstructure.ControllerLayout.cancelRequest = driver.povLeft().or(operator.povLeft());
    Superstructure.ControllerLayout.climbRequest = driver.povRight();
    Superstructure.ControllerLayout.disableTargeting = driver.povUp();
    Superstructure.ControllerLayout.intakeRequest = driver.leftTrigger();
    Superstructure.ControllerLayout.passingRequest = driver.povDown();
    Superstructure.ControllerLayout.joystickX = () -> -driver.getLeftY();
    Superstructure.ControllerLayout.joystickY = () -> -driver.getLeftX();

    superstructure =
        new Superstructure(
            drive,
            intake,
            shooter,
            indexer,
            hood,
            climb,
            drive::getPose,
            drive::getChassisSpeeds,
            drive::getRotation);

    autobuilder = new AutoBuilder(superstructure, drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Climb/SysId", climb.sysId());
    autoChooser.addOption("Intake/SysId", intake.sysId());
    autoChooser.addOption("auto builder", autobuilder.build());
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    operator
        .a()
        .whileTrue(
            Commands.sequence(
                shooter.sysid(8.0, 0, "left"),
                shooter.sysid(8.0, 1, "middle"),
                shooter.sysid(8.0, 2, "right")));

    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driver
        .x()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Pose2d pose = drive.getPose();
                      Pose2d stepped =
                          pose.transformBy(new Transform2d(-0.25, 0.0, Rotation2d.kZero));
                      drive.setPose(stepped);
                      Logger.recordOutput("Shooting/Calib/StepBackPose", stepped);
                    },
                    drive)
                .ignoringDisable(true));

    operator
        .x()
        .whileTrue(
            Commands.run(
                () -> {
                  double rpm = calibRpm.get();
                  double angleDeg = calibAngleDeg.get();
                  double distance =
                      drive
                          .getPose()
                          .getTranslation()
                          .getDistance(FieldConstants.Hub.hubCenter.getTranslation());
                  hood.setAngle(edu.wpi.first.units.Units.Degrees.of(angleDeg));
                  shooter.setVelocitySetpoint(
                      edu.wpi.first.units.Units.RadiansPerSecond.of(
                          Units.rotationsPerMinuteToRadiansPerSecond(rpm)));
                  Logger.recordOutput("Shooting/Calib/DistanceMeters", distance);
                  Logger.recordOutput("Shooting/Calib/Rpm", rpm);
                  Logger.recordOutput("Shooting/Calib/AngleDeg", angleDeg);
                },
                hood,
                shooter));

    driver
        .y()
        .whileTrue(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> {
                      ShootingManager.ShotSolution solution =
                          shootingManager.calculateShotSolution(
                              drive.getPose(),
                              drive.getChassisSpeeds(),
                              FieldConstants.Hub.topCenterPoint,
                              0.1,
                              0.1);
                      Logger.recordOutput(
                          "ShootingManager/Sim/HeadingDeg",
                          solution.drivetrainHeading.getDegrees());
                      return solution.drivetrainHeading;
                    }),
                Commands.run(
                    () -> {
                      shootingManager.updateFromSuppliers();
                      ShootingManager.ShotSolution solution =
                          shootingManager.calculateShotSolution(
                              drive.getPose(),
                              drive.getChassisSpeeds(),
                              FieldConstants.Hub.topCenterPoint,
                              0.1,
                              0.1);
                      Translation3d[] trajectory =
                          shootingManager.calculateTrajectory(
                              drive.getPose(),
                              solution,
                              FieldConstants.Hub.topCenterPoint,
                              0.02,
                              2.0);
                      Logger.recordOutput("ShootingManager/Sim/Trajectory", trajectory);
                      Logger.recordOutput(
                          "ShootingManager/Sim/HoodAngleDeg",
                          Units.radiansToDegrees(solution.hoodPitchRad));
                      Logger.recordOutput("ShootingManager/Sim/Rpm", solution.flywheelRpm);
                      Logger.recordOutput(
                          "ShootingManager/Sim/HeadingDeg",
                          solution.drivetrainHeading.getDegrees());
                      hood.setAngle(edu.wpi.first.units.Units.Radians.of(solution.hoodPitchRad));
                      shooter.setVelocitySetpoint(
                          edu.wpi.first.units.Units.RadiansPerSecond.of(
                              Units.rotationsPerMinuteToRadiansPerSecond(solution.flywheelRpm)));
                    },
                    hood,
                    shooter)));

    LoggedTunableNumber climbPosition = new LoggedTunableNumber("CLimb/Desired Position", 0.0);
    driver.povLeft().whileTrue(climb.setPositionDynamic(climbPosition));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

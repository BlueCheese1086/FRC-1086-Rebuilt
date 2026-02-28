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

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutosManager;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
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
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.FeederIO.FeederIO;
import frc.robot.subsystems.shooter.FeederIO.FeederIOSim;
import frc.robot.subsystems.shooter.FeederIO.FeederIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.AllianceFlipUtil;
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
@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AutosManager automanager;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Shooter shooter;
  private final Intake intake;

  @SuppressWarnings("unused")
  private final Indexer indexer;

  private final Hood hood;

  @SuppressWarnings("unused")
  private final Climb climb;

  @SuppressWarnings("unused")
  // private final Superstructure superstructure;
  double angle = 0;

  public LoggedTunableNumber hoodAngle;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final CommandXboxController operator = new CommandXboxController(1);

  private Pose2d[] backStartPose = new Pose2d[1];

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("backLeft", VisionConstants.robotToLeftCam),
                new VisionIOPhotonVision("backRight", VisionConstants.robotToRightCam),
                new VisionIOLimelight("limelight-marble", drive::getRotation));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter =
            new Shooter(
                new FeederIOTalonFX(RobotMap.ShooterMap.feeder),
                new ShooterIOTalonFX(RobotMap.ShooterMap.left, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.middle, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.right, false)
                // new FeederIO() {}, new ShooterIO() {}
                );
        hood = new Hood(new HoodIOServo());
        climb = new Climb(new ClimbIO() {});
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("backLeft", VisionConstants.robotToLeftCam),
                new VisionIOPhotonVision("backRight", VisionConstants.robotToRightCam));
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("left", VisionConstants.robotToLeftCam),
                new VisionIOPhotonVision("right", VisionConstants.robotToRightCam));
        shooter = new Shooter(new FeederIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hood = new Hood(new HoodIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    hoodAngle = new LoggedTunableNumber("Hood Setpoint", 55.0);

    automanager = new AutosManager(drive, shooter, indexer, intake, hood);
    AutoRoutines.setup(drive, automanager.machine);
    // Shooting manager uses drive pose/speeds for SOTM calculations
    // shootingManager =
    //     new ShootingManager(drive::getPose, drive::getChassisSpeeds, drive::getRotation);

    Superstructure.ControllerLayout.scoreRequest = driver.rightTrigger();
    Superstructure.ControllerLayout.cancelRequest = driver.povLeft().or(operator.povLeft());
    Superstructure.ControllerLayout.climbRequest = operator.rightTrigger();
    Superstructure.ControllerLayout.disableTargeting = driver.y();
    Superstructure.ControllerLayout.intakeRequest = driver.leftTrigger();
    Superstructure.ControllerLayout.passingRequest = driver.b();
    Superstructure.ControllerLayout.joystickX = () -> -driver.getLeftY();
    Superstructure.ControllerLayout.joystickY = () -> -driver.getLeftX();
    Superstructure.ControllerLayout.driverHid = driver::getHID;

    // superstructure =
    // new Superstructure(
    // drive,
    // intake,
    // shooter,
    // indexer,
    // hood,
    // climb,
    // drive::getPose,
    // drive::getChassisSpeeds,
    // drive::getRotation);
    // autobuilder = new AutoBuilder(superstructure, drive);

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
    autoChooser.addOption("auto builder", automanager.getSelectedAuto());

    autoChooser.addOption("Intake Pivot SysId", intake.sysId());
    autoChooser.addOption("Climb SysId", climb.sysId());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // TODO: REMOVE THIS DURING SUPERSTRUCTURE TESTING
    // Default command must require the subsystem; run periodically to maintain setpoint
    // hood.setDefaultCommand(hood.setAngle(Degrees.of(angle)));
    hood.setDefaultCommand(Commands.run(() -> hood.setPosition(() -> hoodAngle.get()), hood));

    driver
        .start()
        .onTrue(
            Commands.run(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driver
        .leftTrigger()
        .and(()-> driver.leftBumper())
        .whileTrue(
            Commands.parallel(
                // DriveCommands.joystickDriveSyom(
                //     drive, ControllerLayout.joystickX, ControllerLayout.joystickY),
                intake.setVoltage(IntakeConstants.Setpoints.run),
                intake.setPosition(IntakeConstants.Setpoints.deployed)));

    driver
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                shooter
                    .runFeederVoltage(ShooterConstants.FeederSetpoints.run.in(Volts))
                    .finallyDo(shooter.runFeederVoltage(0.0)::execute),
                indexer.setVoltage(IndexerConstants.Setpoints.feed)));
    driver.rightBumper().whileTrue(DriveCommands.recordData(drive, shooter, hood));
    driver
        .povLeft()
        .whileTrue(
            Commands.parallel(
                intake.setVoltage(IntakeConstants.Setpoints.run.negate()),
                indexer.setVoltage(IndexerConstants.Setpoints.feed.negate()),
                shooter
                    .runFeederVoltage(-ShooterConstants.FeederSetpoints.run.in(Volts))
                    .finallyDo(shooter.runFeederVoltage(0.0)::execute)));
    // Commands.run(() -> shooter.setVoltage(12))));
    driver
        .povDown()
        .whileTrue(
            Commands.parallel(
                indexer.setVoltage(IndexerConstants.Setpoints.intake.negate()),
                shooter
                    .runFeederVoltage(-ShooterConstants.FeederSetpoints.run.in(Volts))
                    .finallyDo(shooter.runFeederVoltage(0.0)::execute),
                Commands.run(() -> shooter.setVoltage(12))));

    // Operator Commands
    driver.y().onTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));
    operator.povRight().onTrue(intake.setPosition(IntakeConstants.Setpoints.agitate));
    operator.x().whileTrue(DriveCommands.recordData(drive, shooter, hood));
    driver
        .a()
        .whileTrue(
            shooter.setVelocity(
                () -> (RadiansPerSecond.of(ShooterConstants.Tuning.velocitySetpoint.get()))));
    operator
        .povDown()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> backStartPose[0] = drive.getPose()),
                    Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0.0, 0.0)), drive)
                    Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0.0, 0.0)), drive)
                        .until(
                            () ->
                                backStartPose[0] != null
                                    && drive
                                            .getPose()
                                            .getTranslation()
                                            .getDistance(backStartPose[0].getTranslation())
                                        >= Units.inchesToMeters(5)))
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())));
  }

  public void logDist() {
    Logger.recordOutput(
        "Regression/Distance to Center Shooter",
        shooter
            .getShooterPoses(drive.getPose())[1]
            .toPose2d()
            .relativeTo(FieldConstants.Hub.hubCenter)
            .getTranslation()
            .getNorm());
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

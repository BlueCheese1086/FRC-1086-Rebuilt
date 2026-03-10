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

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.Autos;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.PathPlannerCommands;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
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
import frc.robot.subsystems.shooter.ShooterConstants.FeederSetpoints;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTransforms;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator.LaunchingParameters;
import frc.robot.subsystems.shooter.shooterUtil.ShootingManager;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
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

  private final ShootingManager shootingManager;
  private final PathPlannerCommands ppCommands;

  @SuppressWarnings("unused")
  private Supplier<Rotation2d> driveAngle = () -> Rotation2d.kZero;

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

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    "backLeft", VisionConstants.robotToLeftCam, drive::getRotation),
                new VisionIOPhotonVision(
                    "backRight", VisionConstants.robotToRightCam, drive::getRotation),
                new VisionIOLimelight("limelight-marble", drive::getRotation));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter =
            new Shooter(
                new FeederIOTalonFX(RobotMap.ShooterMap.feeder),
                new ShooterIOTalonFX(RobotMap.ShooterMap.left, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.middle, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.right, false));
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
                new VisionIOPhotonVisionSim(
                    "backLeft", VisionConstants.robotToLeftCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "backRight", VisionConstants.robotToRightCam, drive::getPose));
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
                new VisionIOPhotonVision(
                    "left", VisionConstants.robotToLeftCam, drive::getRotation),
                new VisionIOPhotonVision(
                    "right", VisionConstants.robotToRightCam, drive::getRotation));
        shooter = new Shooter(new FeederIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hood = new Hood(new HoodIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }
    
    Autos.setup(drive, intake);
    automanager = new AutosManager(drive, shooter, indexer, intake, hood);
    AutoRoutines.setup(drive, automanager.machine);
    // Shooting manager uses drive pose/speeds for SOTM calculations
    shootingManager =
        new ShootingManager(drive::getPose, drive::getChassisSpeeds, drive::getRotation);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    ppCommands = new PathPlannerCommands();
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
    autoChooser.addOption("Shooter Sys id", shooter.sysid(5.0, 0, "shooter"));
    autoChooser.addDefaultOption("left Side Auto", this.pathFindToStart("left"));
    // autoChooser.addOption("right Side Auto", this.pathFindToStart("rightauto"));

    NamedCommands.registerCommand("IntakeRun", intake.setVoltage(IntakeConstants.Setpoints.run));
    new PathPlannerAuto("left")
        .event("Shoot")
        .onTrue(ppCommands.aimAndShoot(drive, shooter, hood, indexer))
        .onFalse(Commands.runOnce(shooter::stopAll));
    new PathPlannerAuto("left")
        .event("IntakeDown")
        .onTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));

    // // autoChooser.addOption("Swiper's Auto", new PathPlannerAuto("Swiper
    // Auto"));
    // NamedCommands.registerCommand(
    // "IntakeDown",
    // intake.setPosition(IntakeConstants.Setpoints.deployed).withTimeout(0.5));
    // NamedCommands.registerCommand(
    // "IntakeUp",
    // intake.setPosition(IntakeConstants.Setpoints.stowed).withTimeout(0.5));
    // NamedCommands.registerCommand(
    // "IntakeRun",
    // intake.setVoltage(IntakeConstants.Setpoints.run).withTimeout(2.4));
    // NamedCommands.registerCommand(
    // "TimedIntakeRun",
    // intake.setVoltage(IntakeConstants.Setpoints.run).withTimeout(1.5));
    // NamedCommands.registerCommand(
    // "Shoot",
    // shooter
    // .runShooter(() -> RadiansPerSecond.of(350))
    // .withTimeout(1.5)); // Broken: Crazy Oscilation + ending/averaging @ 100
    // velocity for
    // some reason
    // NamedCommands.registerCommand(
    // "Feed",
    // shooter
    // .runFeed(ShooterConstants.FeederSetpoints.run.in(Volts))
    // .withTimeout(1.5));
    // NamedCommands.registerCommand(
    // "Index",
    // indexer
    // .setVoltage(IndexerConstants.Setpoints.feed)
    // .withTimeout(1.5));
    // NamedCommands.registerCommand(
    // "Climb",
    // AutoClimb()); //Climb is not working :(
    // NamedCommands.registerCommand("IntakeStop",
    // intake.setVoltage(IntakeConstants.Setpoints.stop));

    NamedCommands.registerCommand("Climb", Commands.none());
    NamedCommands.registerCommand("IntakeUp", Commands.none());
    NamedCommands.registerCommand("IntakeDown", Commands.none());
    NamedCommands.registerCommand("IntakeStop", Commands.none());
    NamedCommands.registerCommand("IntakeRun", Commands.none());
    NamedCommands.registerCommand("Index", Commands.none());
    NamedCommands.registerCommand("Feed", Commands.none());
    NamedCommands.registerCommand("Shoot", Commands.none());
    NamedCommands.registerCommand("TimedIntakeRun", Commands.none());
    autoChooser.addOption("Auto Path 1", Autos.runAutonomous("morepaths/Path1"));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    hood.setDefaultCommand(
        Commands.run(
            () -> hood.setPosition(() -> HoodConstants.Targeting.hoodAngle.getAsDouble()), hood));

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                intake.setVoltage(IntakeConstants.Setpoints.run),
                intake.setPosition(IntakeConstants.Setpoints.deployed)));

    driver
        .leftTrigger()
        .and(driver.rightBumper())
        .whileTrue(
            Commands.parallel(
                intake.setVoltage(IntakeConstants.Setpoints.run),
                intake.setPosition(IntakeConstants.Setpoints.deployed),
                DriveCommands.joystickDriveSyom(
                    drive, () -> -driver.getLeftY(), () -> -driver.getLeftX())));

    driver
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                shooter.runFeed(FeederSetpoints.run.in(Volts)),
                indexer.setVoltage(IndexerConstants.Setpoints.feed),
                intake.setVoltage(IntakeConstants.Setpoints.run),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    Commands.repeatingSequence(
                        intake.setPosition(IntakeConstants.Setpoints.agitate),
                        Commands.waitSeconds(0.2),
                        intake.setPosition(IntakeConstants.Setpoints.deployed),
                        Commands.waitSeconds(0.2)))));

    driver
        .povRight()
        .whileTrue(
            Commands.parallel(
                intake.setVoltage(IntakeConstants.Setpoints.run.unaryMinus()),
                indexer.setVoltage(IndexerConstants.Setpoints.feed.unaryMinus()),
                shooter.runFeed(FeederSetpoints.run.unaryMinus().in(Volts)),
                shooter.setVoltage(12.0).finallyDo(shooter::stopShooter)));
    driver
        .povDown()
        .whileTrue(
            Commands.parallel(
                indexer.setVoltage(IndexerConstants.Setpoints.feed.unaryMinus()),
                shooter.setVoltage(12.0),
                shooter.runFeed(-12.0).finallyDo(shooter::stopShooter)));

    // driver
    // .povUp()
    // .multiPress(2, 1.0)
    // .whileTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));

    driver.povUp().whileTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));

    // Known Tower Shot
    driver
        .a()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                        () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(350.0)),
                        shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.runOnce(() -> hood.setPosition(() -> 64.0))));

    // Passing
    driver
        .b()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                        () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(350.0)),
                        shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.runOnce(() -> hood.setPosition(() -> 54.0))));

    driver
        .y()
        .whileTrue(
            Commands.parallel(
                    Commands.run(
                        () -> {
                          LaunchingParameters parms =
                              LauncherCalculator.getInstance()
                                  .getParameters(
                                      () ->
                                          (new Pose3d(drive.getPose())
                                              .transformBy(ShooterTransforms.centerShooter)
                                              .toPose2d()),
                                      drive::getChassisSpeeds,
                                      drive::getRotation);
                          shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(350.0));
                          shooter.setVelocitySetpoint(
                              () -> RadiansPerSecond.of(parms.flywheelSpeed()));
                          hood.setPosition(() -> parms.hoodAngle());

                          Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
                          Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
                          Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
                        },
                        shooter),
                    DriveCommands.joystickDriveLockRadiusToTarget(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> FieldConstants.Hub.hubCenter,
                        () ->
                            LauncherCalculator.getInstance()
                                .getParameters(
                                    () ->
                                        new Pose3d(drive.getPose())
                                            .transformBy(ShooterTransforms.centerShooter)
                                            .toPose2d(),
                                    drive::getChassisSpeeds,
                                    drive::getRotation)
                                .driveAngle()))
                .finallyDo(shooter::stopShooter));
    driver
        .x()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                      var sol =
                          shootingManager.calculateShotSolution(
                              drive.getPose(),
                              drive.getChassisSpeeds(),
                              FieldConstants.Hub.topCenterPoint,
                              0.10,
                              0.10);
                      shooter.setVelocitySetpoint(
                          () -> RotationsPerSecond.of(sol.flywheelRpm / 60));
                      hood.setPosition(() -> sol.hoodPitchRad);
                    },
                    shooter),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () ->
                        shootingManager.calculateShotSolution(
                                drive.getPose(),
                                drive.getChassisSpeeds(),
                                FieldConstants.Hub.topCenterPoint,
                                0.10,
                                0.10)
                            .drivetrainHeading)));

    // Operator Commands
    operator.leftTrigger().onTrue(intake.setVoltage(IntakeConstants.Setpoints.run));
    operator
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    shooter.setVelocitySetpoint(
                        () ->
                            (RadiansPerSecond.of(
                                ShooterConstants.Tuning.velocitySetpoint.getAsDouble()))),
                shooter))
        .onFalse(Commands.runOnce(shooter::stopShooter));

    operator.b().whileTrue(indexer.setVoltage(IndexerConstants.Setpoints.feed));
    operator.y().whileTrue(shooter.runFeed(ShooterConstants.FeederSetpoints.run.in(Volts)));
    operator.leftBumper().whileTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));
    operator.rightBumper().whileTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));
    operator.povUp().onTrue(climb.setPosition(ClimbConstants.extendedHeight));
    operator.povDown().onTrue(climb.setPosition(ClimbConstants.retractedHeight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command pathFindToStart(String pathName) {
    Command pathFind =
        AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(new PathPlannerAuto(pathName).getStartingPose()),
            new PathConstraints(
                MetersPerSecond.of(drive.getMaxLinearSpeedMetersPerSec()),
                MetersPerSecondPerSecond.of(Math.pow(drive.getMaxLinearSpeedMetersPerSec(), 2)),
                RadiansPerSecond.of(drive.getMaxAngularSpeedRadPerSec()),
                RadiansPerSecondPerSecond.of(Math.pow(drive.getMaxAngularSpeedRadPerSec(), 2)),
                Volts.of(RobotController.getBatteryVoltage())));
    return Commands.sequence(pathFind, new PathPlannerAuto(pathName));
  }
}

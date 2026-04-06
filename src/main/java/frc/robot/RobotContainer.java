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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.Autos;
import frc.robot.autonomous.AutosManager;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.BatteryLogger;
import frc.robot.util.FieldConstants;
import frc.robot.util.MatchTimer;
import frc.robot.util.PoseMath;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  private final ShootingManager shootingManager;
  private final BatteryLogger batteryLogger = new BatteryLogger();

  @SuppressWarnings("unused")
  private Supplier<Rotation2d> driveAngle = () -> Rotation2d.kZero;

  private boolean manualOverride = false;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final CommandXboxController operator = new CommandXboxController(1);

  // private Pose2d[] backStartPose = new Pose2d[1];

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Alliance startingAlliance = Alliance.Red;

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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                batteryLogger);

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    "backLeft", VisionConstants.robotToLeftCam, drive::getRotation),
                new VisionIOPhotonVision(
                    "backRight", VisionConstants.robotToRightCam, drive::getRotation),
                new VisionIOLimelight("limelight-marble", drive::getRotation));

        intake = new Intake(new IntakeIOTalonFX(), batteryLogger);
        indexer = new Indexer(new IndexerIOTalonFX(), batteryLogger);
        shooter =
            new Shooter(
                new FeederIOTalonFX(RobotMap.ShooterMap.feeder),
                new ShooterIOTalonFX(
                    RobotMap.ShooterMap.left, true, ShooterConstants.Tuning.leftShooterConfigs),
                new ShooterIOTalonFX(
                    RobotMap.ShooterMap.middle, true, ShooterConstants.Tuning.middleShooterConfigs),
                new ShooterIOTalonFX(
                    RobotMap.ShooterMap.right, false, ShooterConstants.Tuning.rightShooterConfigs));
        hood = new Hood(new HoodIOServo());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                batteryLogger);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "backLeft", VisionConstants.robotToLeftCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "backRight", VisionConstants.robotToRightCam, drive::getPose));
        indexer = new Indexer(new IndexerIOSim(), batteryLogger);
        intake = new Intake(new IntakeIOSim(), batteryLogger);
        shooter =
            new Shooter(
                new FeederIOSim(), new ShooterIOSim(), new ShooterIOSim(), new ShooterIOSim());
        hood = new Hood(new HoodIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                batteryLogger);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    "backLeft", VisionConstants.robotToLeftCam, drive::getRotation),
                new VisionIOPhotonVision(
                    "backRight", VisionConstants.robotToRightCam, drive::getRotation),
                new VisionIO() {});
        shooter = new Shooter(new FeederIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {}, batteryLogger);
        indexer = new Indexer(new IndexerIO() {}, batteryLogger);
        hood = new Hood(new HoodIO() {});
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
    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("auto builder", automanager.getSelectedAuto());

    // autoChooser.addOption("Intake Pivot SysId", intake.sysId());
    // autoChooser.addOption("Climb SysId", climb.sysId());
    // autoChooser.addOption("Shooter Sys id", shooter.sysid(5.0, 0, "shooter"));
    autoChooser.addOption("Left Bump Auto", this.pathFindToStart("left1", false));
    autoChooser.addOption("Right Bump Auto", this.pathFindToStart("left1", true));
    autoChooser.addOption("Right Bump Auto (w/ outpost)", this.pathFindToStart("right1", false));
    autoChooser.addOption("SSHWTMHTCMCBIATSTGTTTW", this.pathFindToStart("BSHIGTTMMCM", false));
    autoChooser.addOption(
        "Basic Shooting Auto",
        Commands.sequence(
            Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              AllianceFlipUtil.shouldFlip() ? 0.5 : -0.5,
                              0.0,
                              0.0,
                              drive.getRotation()));
                    },
                    drive)
                .finallyDo(drive::stop)
                .withTimeout(0.75),
            intake.setPosition(IntakeConstants.Setpoints.deployed),
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
                          shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(365.0));
                          hood.setPosition(() -> 80.0);
                          Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
                          Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
                          Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
                          Logger.recordOutput("Shoot Parms/ Distance", parms.distance());
                        },
                        shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.waitSeconds(1.0)
                    .andThen(
                        Commands.parallel(
                            shooter
                                .runFeed(ShooterConstants.FeederSetpoints.run.in(Volts))
                                .finallyDo(shooter::stopFeeder),
                            indexer.setVoltage(IndexerConstants.Setpoints.feed),
                            Commands.repeatingSequence(
                                intake.setPosition(IntakeConstants.Setpoints.agitate),
                                Commands.waitSeconds(0.2),
                                intake.setPosition(IntakeConstants.Setpoints.deployed),
                                Commands.waitSeconds(0.2)))))));
    autoChooser.addOption("Outpost 1", this.pathFindToStart("outpost", false));
    autoChooser.addOption("Risky right bump auto", this.pathFindToStart("blasty", false));
    autoChooser.addOption("Risky left bump auto", this.pathFindToStart("blasty", true));
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
            () -> {
              if (Constants.tuningMode) {
                hood.setPosition(HoodConstants.Targeting.hoodAngle::get);
              } else {
                LaunchingParameters parms =
                    LauncherCalculator.getInstance()
                        .getParameters(
                            () ->
                                (new Pose3d(drive.getPose())
                                    .transformBy(ShooterTransforms.centerShooter)
                                    .toPose2d()),
                            drive::getChassisSpeeds,
                            drive::getRotation);
                hood.setPosition(() -> parms.hoodAngle());
              }
            },
            hood));

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
                    Commands.waitSeconds(0.67),
                    Commands.repeatingSequence(
                        intake.setPosition(IntakeConstants.Setpoints.agitate),
                        Commands.waitSeconds(0.4),
                        intake.setPosition(IntakeConstants.Setpoints.deployed),
                        Commands.waitSeconds(0.4)))));

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

    driver.povUp().whileTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));

    // Known Tower Shot
    driver
        .a()
        .whileTrue(
            Commands.race(
                Commands.run(
                        () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(370.0)),
                        shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.run(() -> hood.setPosition(() -> 64.0), hood)));

    // What i think is better and safer is a known trench shot. like 1678, they cant
    // be defended
    // there
    // driver
    // .a()
    // .whileTrue(
    // Commands.parallel(
    // Commands.run(
    // () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(385)),
    // shooter)
    // .finallyDo(shooter::stopShooter),
    // Commands.runOnce(() -> hood.setPosition(() -> 60.0))));

    // Passing
    driver
        .b()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                        () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(375.0)),
                        shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.run(() -> hood.setPosition(() -> 54.0), hood)));

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
                          // shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(350.0));
                          shooter.setVelocitySetpoint(
                              () ->
                                  RadiansPerSecond.of(
                                      (PoseMath.getDistanceToTarget(
                                                  drive.getPose(),
                                                  AllianceFlipUtil.apply(
                                                      FieldConstants.Hub.hubCenter))
                                              < Units.inchesToMeters(137.76)
                                          ? 375
                                          : 425)));
                          hood.setPosition(() -> parms.hoodAngle());

                          Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
                          Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
                          Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
                          Logger.recordOutput("Shoot Parms/ Distance", parms.distance());
                        },
                        shooter,
                        hood),
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
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
    driver.x().whileTrue(sotm());

    // Operator Commands
    operator.leftTrigger().onTrue(intake.runVelocity(IntakeConstants.Setpoints.rollerVelocity));
    operator
        .y()
        .whileTrue(
            Commands.run(
                () ->
                    shooter.setVelocitySetpoint(
                        () ->
                            RadiansPerSecond.of(
                                ShooterConstants.Tuning.velocitySetpoint.getAsDouble())),
                shooter))
        .onFalse(Commands.runOnce(shooter::stopShooter));

    operator.b().whileTrue(indexer.setVoltage(IndexerConstants.Setpoints.feed));
    operator.a().whileTrue(shooter.runFeed(ShooterConstants.FeederSetpoints.run.in(Volts)));
    operator.leftBumper().whileTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));
    operator.rightBumper().whileTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));
    // operator.povUp().onTrue(climb.setPosition(ClimbConstants.extendedHeight));
    // operator.povDown().onTrue(climb.setPosition(ClimbConstants.retractedHeight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @AutoLogOutput(key = "Targetting/Distance")
  private double getDist() {
    return new Pose3d(drive.getPose())
        .transformBy(ShooterConstants.ShooterTransforms.centerShooter)
        .toPose2d()
        .relativeTo(FieldConstants.Hub.hubCenter)
        .getTranslation()
        .getNorm();
  }

  public void periodic() {
    Logger.recordOutput(
        "Distance To Hub",
        new Pose3d(drive.getPose())
            .transformBy(ShooterTransforms.centerShooter)
            .relativeTo(new Pose3d(FieldConstants.Hub.topCenterPoint, Rotation3d.kZero))
            .getTranslation()
            .getNorm());
    Logger.recordOutput(
        "Vision Transforms/Left",
        new Pose3d(drive.getPose()).transformBy(VisionConstants.robotToLeftCam));
    Logger.recordOutput(
        "Vision Transforms/Right",
        new Pose3d(drive.getPose()).transformBy(VisionConstants.robotToRightCam));
    String data = DriverStation.getGameSpecificMessage();
    if (data.length() > 0) {
      startingAlliance = data.charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;
    }
    Logger.recordOutput("Hub/Game Data", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("Hub/Active", FieldConstants.Hub.isHubActive(startingAlliance));
    Logger.recordOutput("Hub/Sim Match Time", MatchTimer.getTime());
  }

  public Command pathFindToStart(String pathName, boolean flip) {
    NamedCommands.registerCommand(
        "IntakeDown", intake.setPosition(IntakeConstants.Setpoints.deployed));
    NamedCommands.registerCommand("IntakeRun", intake.setVoltage(IntakeConstants.Setpoints.run));
    NamedCommands.registerCommand(
        "RunupShooter",
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
                  shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(parms.flywheelSpeed()));
                  hood.setPosition(() -> parms.hoodAngle());
                })
            .until(shooter::atSetpoint));
    NamedCommands.registerCommand(
        "LoadUp",
        Commands.parallel(
                shooter.runFeed(ShooterConstants.FeederSetpoints.run.in(Volts)),
                indexer.setVoltage(IndexerConstants.Setpoints.feed),
                intake.setVoltage(IntakeConstants.Setpoints.run))
            .finallyDo(shooter::stopAll));
    NamedCommands.registerCommand("IntakeUp", intake.setPosition(IntakeConstants.Setpoints.stowed));
    NamedCommands.registerCommand("ShootNow", getShootCommand());
    Command pathFind =
        AutoBuilder.pathfindToPose(
            AllianceFlipUtil.apply(new PathPlannerAuto(pathName, flip).getStartingPose()),
            new PathConstraints(
                MetersPerSecond.of(drive.getMaxLinearSpeedMetersPerSec()),
                MetersPerSecondPerSecond.of(Math.pow(drive.getMaxLinearSpeedMetersPerSec(), 2)),
                RadiansPerSecond.of(drive.getMaxAngularSpeedRadPerSec()),
                RadiansPerSecondPerSecond.of(Math.pow(drive.getMaxAngularSpeedRadPerSec(), 2)),
                Volts.of(RobotController.getBatteryVoltage())));
    return Commands.sequence(
        Commands.defer(
            () ->
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(new PathPlannerAuto(pathName, flip).getStartingPose()),
                    new PathConstraints(
                        MetersPerSecond.of(drive.getMaxLinearSpeedMetersPerSec()),
                        MetersPerSecondPerSecond.of(
                            Math.pow(drive.getMaxLinearSpeedMetersPerSec(), 2)),
                        RadiansPerSecond.of(drive.getMaxAngularSpeedRadPerSec()),
                        RadiansPerSecondPerSecond.of(
                            Math.pow(drive.getMaxAngularSpeedRadPerSec(), 2)),
                        Volts.of(RobotController.getBatteryVoltage()))),
            Set.of(drive)),
        new PathPlannerAuto(pathName, flip));
  }

  private Command getShootCommand() {
    return Commands.parallel(
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
                  shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(parms.flywheelSpeed()));
                  hood.setPosition(() -> parms.hoodAngle());
                  Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
                  Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
                  Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
                  Logger.recordOutput("Shoot Parms/Distance", parms.distance());
                },
                shooter)
            .finallyDo(shooter::stopShooter),
        Commands.waitSeconds(1.0)
            .andThen(
                Commands.parallel(
                    shooter
                        .runFeed(ShooterConstants.FeederSetpoints.run.in(Volts))
                        .finallyDo(shooter::stopFeeder),
                    indexer.setVoltage(IndexerConstants.Setpoints.feed),
                    intake.setVoltage(IntakeConstants.Setpoints.run))),
        Commands.repeatingSequence(
            intake.setPosition(IntakeConstants.Setpoints.agitate),
            Commands.waitSeconds(0.2),
            intake.setPosition(IntakeConstants.Setpoints.deployed),
            Commands.waitSeconds(0.2)));
  }

  private Command sotm() {
    return Commands.parallel(
        Commands.run(
                () -> {
                  var sol =
                      shootingManager.calculateShotSolution(
                          drive.getPose(),
                          drive.getChassisSpeeds(),
                          FieldConstants.Hub.topCenterPoint,
                          0.10,
                          0.10);
                  shooter.setVelocitySetpoint(() -> RotationsPerSecond.of(sol.flywheelRpm / 60));
                  hood.setPosition(() -> Units.radiansToDegrees(sol.hoodPitchRad));
                },
                shooter)
            .finallyDo(shooter::stopShooter),
        DriveCommands.joystickDriveAtAngleFast(
            drive,
            () -> -driver.getLeftY() * 0.5, // Limit translation speed during SOTM
            () -> -driver.getLeftX() * 0.5,
            () ->
                shootingManager.calculateShotSolution(
                        drive.getPose(),
                        drive.getChassisSpeeds(),
                        FieldConstants.Hub.topCenterPoint,
                        0.10,
                        0.10)
                    .drivetrainHeading),
        Commands.waitSeconds(0.75)
            .andThen(
                Commands.parallel(
                    shooter
                        .runFeed(ShooterConstants.FeederSetpoints.run.in(Volts))
                        .finallyDo(shooter::stopFeeder),
                    indexer.setVoltage(IndexerConstants.Setpoints.feed),
                    intake.setVoltage(IntakeConstants.Setpoints.run))),
        Commands.repeatingSequence(
            intake.setPosition(IntakeConstants.Setpoints.agitate),
            Commands.waitSeconds(0.4),
            intake.setPosition(IntakeConstants.Setpoints.deployed),
            Commands.waitSeconds(0.4)));
  }
}

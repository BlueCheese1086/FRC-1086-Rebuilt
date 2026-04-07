package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.Autos;
import frc.robot.autonomous.AutosManager;
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
import frc.robot.subsystems.shooter.shooterUtil.PassingManager;
import frc.robot.subsystems.shooter.shooterUtil.PassingManager.PassingParams;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.BatteryLogger;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesVertical;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.controllers.OverrideSwitches;

import frc.robot.util.MatchTimer;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AutosManager automanager;
  private final AutoRoutines factory;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Shooter shooter;
  private final Intake intake;

  @SuppressWarnings("unused")
  private final Indexer indexer;

  private final Hood hood;

  private final BatteryLogger batteryLogger = new BatteryLogger();

  @SuppressWarnings("unused")
  private Supplier<Rotation2d> driveAngle = () -> Rotation2d.kZero;

  private boolean manualOverride = false;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController testing = new CommandXboxController(2);
  private final OverrideSwitches overrides = new OverrideSwitches(5);

  // Operator overrides
  private final Trigger robotRelative = overrides.operatorSwitch(1);
  private final Trigger coast = overrides.operatorSwitch(2);
  private final Trigger lostAutoOverride = overrides.multiDirectionSwitchLeft();
  private final Trigger wonAutoOverride = overrides.multiDirectionSwitchRight();
  private final Trigger ignoreHubState = overrides.operatorSwitch(1);

  // Alerts
  private final Alert driverDisconnected =
      new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final Alert autoWinnerNotSet = new Alert("!!! AUTO WINNER NOT SET !!!", AlertType.kError);
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public Alliance startingAlliance = Alliance.Blue;

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
                new ShooterIOTalonFX(RobotMap.ShooterMap.left, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.middle, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.right, false));
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
    factory = new AutoRoutines(drive, shooter, intake, indexer, hood);
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
    autoChooser.addOption(
        "AutoRoutines Sys ID Rotation", frc.robot.commands.AutoRoutines.autoRotationSysId(drive));
    autoChooser.addOption(
        "AutoRoutines Sys ID Translation",
        frc.robot.commands.AutoRoutines.autoTranslationSysId(drive));
    // autoChooser.addOption("Intake Pivot SysId", intake.sysId());
    // autoChooser.addOption("Climb SysId", climb.sysId());
    // autoChooser.addOption("Shooter Sys id", shooter.sysid(5.0, 0, "shooter"));
    autoChooser.addOption("Left Bump Auto", this.pathFindToStart("left1", false));
    autoChooser.addOption("Right Bump Auto", this.pathFindToStart("left1", true));
    autoChooser.addOption("Right Bump Auto (w/ outpost)", this.pathFindToStart("right1", false));
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
    autoChooser.addOption(
        "Risky right bump auto", this.pathFindToStart("Risky Bump Double Intake", false));
    autoChooser.addOption(
        "Risky left bump auto", this.pathFindToStart("Risky Bump Double Intake", true));
    autoChooser.addOption("Depot Auto", this.pathFindToStart("Depot Auto", false));
    autoChooser.addOption("Testing", this.pathFindToStart("2 Cycle", false));
    autoChooser.addOption("Left Bump Choreo Auto", factory.getLBAuto());
    autoChooser.addOption("Right Bump Choreo Auto", factory.getRBAuto());
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
                if (FieldConstants.LinesVertical.inAllianceZone(drive.getPose())) {
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
              }
            },
            hood));

    shooter.setDefaultCommand(
        shooter
            .run(
                () -> {
                  if (FieldConstants.LinesVertical.inAllianceZone(drive.getPose())) {
                    shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(150.0));
                  } else {
                    shooter.stopShooter();
                  }
                })
            .onlyIf(DriverStation::isTeleop));

    Trigger hubActive =
        new Trigger(
            () ->
                HubShiftUtil.getShiftedShiftInfo().active());

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
        .and(() -> LauncherCalculator.getInstance().getParameters(() ->
                                  (new Pose3d(drive.getPose())
                                      .transformBy(ShooterTransforms.centerShooter)
                                      .toPose2d()),
                              drive::getChassisSpeeds,
                              drive::getRotation).isValid())
        .and(() -> ignoreHubState.getAsBoolean() || hubActive.getAsBoolean())
        .whileTrue(
            Commands.parallel(
                shooter.runFeed(FeederSetpoints.run.in(Volts)),
                indexer.setVoltage(IndexerConstants.Setpoints.feed),
                intake.setVoltage(IntakeConstants.Setpoints.run),
                Commands.sequence(
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
                shooter.setVoltage(8.0).finallyDo(shooter::stopShooter)));
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
            Commands.parallel(
                Commands.run(
                        () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(370.0)),
                        shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.run(() -> hood.setPosition(() -> 64.0), hood)));

    // What i think is better and safer is a known trench shot. like 1678, they cant
    // be defended
    // there
    driver
        .x()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                        () -> shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(385)), shooter)
                    .finallyDo(shooter::stopShooter),
                Commands.runOnce(() -> hood.setPosition(() -> 60.0))));

    // Passing
    driver
        .b()
        .whileTrue(
            Commands.parallel(
                    Commands.run(
                        () -> {
                          PassingParams parms =
                              PassingManager.getInstance()
                                  .getParameters(
                                      () ->
                                          (new Pose3d(drive.getPose())
                                              .transformBy(ShooterTransforms.centerShooter)
                                              .toPose2d()),
                                      drive::getChassisSpeeds,
                                      drive::getRotation);
                          shooter.setVelocitySetpoint(
                              () -> RadiansPerSecond.of(parms.flywheelSpeed()));
                          hood.setPosition(() -> parms.hoodAngle());
                          Logger.recordOutput("Pass Parms/ Hood Angle", parms.hoodAngle());
                          Logger.recordOutput("Pass Parms/ Drive Angle", parms.driveAngle());
                          Logger.recordOutput("Pass Parms/ Flywheel Speed", parms.flywheelSpeed());
                          Logger.recordOutput("Pass Parms/ Distance", parms.distance());
                        },
                        shooter,
                        hood),
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driver.getLeftY() * 0.5,
                        () -> -driver.getLeftX() * 0.5,
                        () ->
                            PassingManager.getInstance()
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
                          shooter.setVelocitySetpoint(
                              () -> RadiansPerSecond.of(parms.flywheelSpeed()));
                          Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
                          Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
                          Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
                          Logger.recordOutput("Shoot Parms/ Distance", parms.distance());
                        },
                        shooter),
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driver.getLeftY() * 0.55,
                        () -> -driver.getLeftX() * 0.55,
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

    // Operator Commands
    operator.leftTrigger().onTrue(intake.setVoltage(IntakeConstants.Setpoints.run));
    operator
        .b()
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
    operator.y().whileTrue(shooter.runFeed(ShooterConstants.FeederSetpoints.run.in(Volts)));
    operator.leftBumper().whileTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));
    operator.rightBumper().whileTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));
    operator
        .povLeft()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      // Lost Auto
                      this.startingAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
                    })
                .ignoringDisable(true));
    operator
        .povRight()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      // Win Auto
                      this.startingAlliance =
                          DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)
                              ? Alliance.Blue
                              : Alliance.Blue;
                    })
                .ignoringDisable(true));
    // ****** ALERTS ******

    // Warn formissing game data
    // Timer teleopElapsedTimer = new Timer();
    // RobotModeTriggers.teleop()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               teleopElapsedTimer.restart();
    //             }));
    // RobotModeTriggers.teleop()
    //     .and(() -> !(DriverStation.getGameSpecificMessage().length() > 0))
    //     .and(() -> HubShiftUtil.getAllianceWinOverride().isEmpty())
    //     .and(() -> teleopElapsedTimer.hasElapsed(1.0))
    //     .whileTrue(
    //         Commands.runEnd(
    //             () -> {
    //               primary.setRumble(RumbleType.kBothRumble, 1);
    //               secondary.setRumble(RumbleType.kBothRumble, 1);
    //             },
    //             () -> {
    //               primary.setRumble(RumbleType.kBothRumble, 0);
    //               secondary.setRumble(RumbleType.kBothRumble, 0);
    //             }))
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> {
    //               autoWinnerNotSet.set(true);
    //               leds.autoWinnerNotSet = true;
    //             },
    //             () -> {
    //               autoWinnerNotSet.set(false);
    //               leds.autoWinnerNotSet = false;
    //             }));

    // // End-of-shift warning
    // for (int i = 1; i <= 5; i++) {
    //   double time = i;
    //   Trigger shiftAboutToEnd =
    //       new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
    //   shiftAboutToEnd
    //       .and(RobotModeTriggers.teleop())
    //       .and(ignoreHubState.negate())
    //       .onTrue(
    //           Commands.runEnd(
    //                   () -> primary.setRumble(RumbleType.kRightRumble, 1.0),
    //                   () -> primary.setRumble(RumbleType.kBothRumble, 0.0))
    //               .withTimeout(0.25));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(driver.getHID().getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operator.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());
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

  @AutoLogOutput(key = "Targetting/DistanceToDS")
  private double getDistToDS() {
    return new Pose3d(drive.getPose())
        .transformBy(ShooterConstants.ShooterTransforms.centerShooter)
        .toPose2d()
        .relativeTo(FieldConstants.defaultAprilTagType.getTagPose(32).get().toPose2d())
        .getTranslation()
        .getNorm();
  }

  public void periodic() {
    Logger.recordOutput(
        "Vision Transforms/Left",
        new Pose3d(drive.getPose()).transformBy(VisionConstants.robotToLeftCam));
    Logger.recordOutput(
        "Vision Transforms/Right",
        new Pose3d(drive.getPose()).transformBy(VisionConstants.robotToRightCam));
    Logger.recordOutput("Robot/In Alliance Zone", LinesVertical.inAllianceZone(drive.getPose()));
    Logger.recordOutput("Robot/Should Flip", AllianceFlipUtil.shouldFlip());
    Logger.recordOutput(
        "Robot/Alliance Zone Red", drive.getPose().getX() >= Units.inchesToMeters(468.0));
    Logger.recordOutput(
        "Robot/Alliance Zone Blue", drive.getPose().getX() >= Units.inchesToMeters(182.11));
    Logger.recordOutput(
        "Match Timer/Can Shoot",
        MatchTimer.hubActiveInTof(
            DriverStation.getAlliance().orElse(Alliance.Blue), startingAlliance, drive.getPose()));
  }

  public Command pathFindToStart(String pathName, boolean flip) {
    NamedCommands.registerCommand(
        "IntakeDown", intake.setPosition(IntakeConstants.Setpoints.deployed));
    NamedCommands.registerCommand("IntakeRun", intake.setVoltage(IntakeConstants.Setpoints.run));
    NamedCommands.registerCommand("IntakeUp", intake.setPosition(IntakeConstants.Setpoints.stowed));
    NamedCommands.registerCommand("ShootNow", getShootCommand());
    NamedCommands.registerCommand("SOTM", getSOTMCommand());
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

  private Command getSOTMCommand() {
    return Commands.parallel(
        getShootCommand(),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> driver.getLeftX(),
            () -> driver.getLeftY(),
            () ->
                LauncherCalculator.getInstance()
                    .getParameters(
                        () ->
                            (new Pose3d(drive.getPose())
                                .transformBy(ShooterTransforms.centerShooter)
                                .toPose2d()),
                        drive::getChassisSpeeds,
                        drive::getRotation)
                    .driveAngle()));
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
                shooter,
                hood)
            .finallyDo(shooter::stopShooter),
        Commands.waitSeconds(0.5)
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
}

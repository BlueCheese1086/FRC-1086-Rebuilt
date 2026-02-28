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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShotCalc;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
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
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShootingManager;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FieldConstants.Hub;
import frc.robot.util.PoseMath;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.util.FieldConstants;
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
  // private final AutosManager automanager;

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
  //   private final Superstructure superstructure;
  // private final Superstructure superstructure;
  private final ShootingManager shootingManager;

  // private final AutoBuilder autobuilder;
  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  private final CommandXboxController operator = new CommandXboxController(1);

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
                drive::getPose,
                // new VisionIOPhotonVision(
                // "left", VisionConstants.PhysicalConstants.cameraTransforms[0]),
                // new VisionIOPhotonVision(
                // "right", VisionConstants.PhysicalConstants.cameraTransforms[1]),
                new VisionIOLimelight("marble"));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter =
            new Shooter(
                new FeederIOTalonFX(RobotMap.ShooterMap.feeder),
                new ShooterIOTalonFX(RobotMap.ShooterMap.left, true),
                new ShooterIOTalonFX(RobotMap.ShooterMap.middle, true),
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
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

        vision =
            new Vision(
                drive::addVisionMeasurement, drive::getPose, new VisionIO() {}, new VisionIO() {});

        shooter = new Shooter(new FeederIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hood = new Hood(new HoodIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Shooting manager uses drive pose/speeds for SOTM calculations
    shootingManager =
        new ShootingManager(drive::getPose, drive::getChassisSpeeds, drive::getRotation);

    // automanager = new AutosManager(drive, shooter, indexer, intake);

    // AutoRoutines.setup(drive, automanager.machine);

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
    //     new Superstructure(
    //         drive,
    //         intake,
    //         shooter,
    //         indexer,
    //         hood,
    //         climb,
    //         drive::getPose,
    //         drive::getChassisSpeeds,
    //         drive::getRotation);
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
    // autoChooser.addOption("auto builder", automanager.getSelectedAuto());

    autoChooser.addOption("Intake Pivot SysId", intake.sysId());
    autoChooser.addOption("Climb SysId", climb.sysId());
    // autoChooser.addOption("auto builder", autobuilder.build());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () ->
    -driver.getRightX()));

    // // TODO: REMOVE THIS DURING SUPERSTRUCTURE TESTING
    // hood.setDefaultCommand(
    //     hood.setAngle(() -> (Degrees.of(HoodConstants.Setpoints.hoodAngle.get()))));
    // TODO: REMOVE THIS DURING SUPERSTRUCTURE TESTING
    // Default command must require the subsystem; run periodically to maintain setpoint
    hood.setDefaultCommand(
        Commands.run(
            () -> hood.setAngle(Degrees.of(HoodConstants.Setpoints.hoodAngle.get())), hood));


    driver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                intake.setVoltage(IntakeConstants.Setpoints.run),
                intake.setPosition(IntakeConstants.Setpoints.deployed)));
    driver.povDown().whileTrue(DriveCommands.moveBack(drive));

    driver.y().whileTrue(Commands.parallel(
                shooter.setVelocity(
                    () -> {
                      return RPM.of(
                          ShotCalc.getShot(
                                  Meters.of(
                                      PoseMath.getDistanceToTarget(drive.getPose(), Hub.hubCenter)))
                              .shooterRPM);
                    }),
                hood.setPosition(
                    () -> {
                      return ShotCalc.getShot(
                              Meters.of(
                                  PoseMath.getDistanceToTarget(drive.getPose(), Hub.hubCenter)))
                          .hoodPosition;
                    })));

    driver.rightTrigger().whileTrue(indexer.setVoltage(IndexerConstants.Setpoints.feed));

    // Operator Commands
    operator
        .rightTrigger()
        .onTrue(
            climb
                .setPosition(ClimbConstants.extendedHeight)
                .until(() -> climb.atSetpoint())
                .andThen(climb.setPosition(ClimbConstants.retractedHeight)));

    // Operator X -> Shoot-on-the-move test (hold to aim and spin up). This also locks
    // rotation to the computed SOTM heading while held (driver retains translation control).
    driver
        .x()
        .whileTrue(
            Commands.parallel(
                // (1) continuously update shooter and hood setpoints
                Commands.run(
                    () -> {
                      var sol =
                          shootingManager.calculateShotSolution(
                              drive.getPose(),
                              drive.getChassisSpeeds(),
                              FieldConstants.Hub.topCenterPoint,
                              0.10,
                              0.10);
                      shooter.setVelocitySetpoint(RotationsPerSecond.of(sol.flywheelRpm / 60.0));
                      hood.setAngle(Degrees.of(Math.toDegrees(sol.hoodPitchRad)));
                    },
                    shooter,
                    hood),

                // (2) keep driver translation but force rotation to the SOTM heading
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

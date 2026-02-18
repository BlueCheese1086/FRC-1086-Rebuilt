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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.hood.HoodConstants;
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
import frc.robot.subsystems.shooter.FeederIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.shooterUtil.ShootingCalculator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FieldConstants;
import frc.robot.util.shooter.LauncherCalculator;
import frc.robot.util.shooter.LauncherCalculator.LaunchingParameters;
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

  @SuppressWarnings("unused")
  private final Superstructure superstructure;

  private final AutoBuilder autobuilder;

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
                new VisionIOPhotonVision(
                    "left", VisionConstants.PhysicalConstants.cameraTransforms[0]),
                new VisionIOPhotonVision(
                    "right", VisionConstants.PhysicalConstants.cameraTransforms[1]),
                new VisionIOLimelight("scoring"));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter =
            new Shooter(
                new FeederIOTalonFX(1),
                new ShooterIOTalonFX(RobotMap.ShooterMap.left),
                new ShooterIOTalonFX(RobotMap.ShooterMap.middle),
                new ShooterIOTalonFX(RobotMap.ShooterMap.right));
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
                new FeederIO() {}, new ShooterIOSim(), new ShooterIOSim(), new ShooterIOSim());
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

    AutoRoutines.setup(drive);

    Superstructure.ControllerLayout.scoreRequest = driver.rightTrigger();
    Superstructure.ControllerLayout.cancelRequest = driver.povLeft().or(operator.povLeft());
    Superstructure.ControllerLayout.climbRequest = driver.povRight();
    Superstructure.ControllerLayout.disableTargeting = driver.povUp();
    Superstructure.ControllerLayout.intakeRequest = driver.leftTrigger();
    Superstructure.ControllerLayout.passingRequest = driver.povDown();
    Superstructure.ControllerLayout.joystickX = () -> -driver.getLeftY();
    Superstructure.ControllerLayout.joystickY = () -> -driver.getLeftX();

    autobuilder = new AutoBuilder(drive); // TODO: pass superstructure when ready
    superstructure = new Superstructure(drive, intake, shooter, indexer, hood, climb, autobuilder, drive::getPose,drive::getChassisSpeeds,drive::getRotation);

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

    // Reset gyro to 0° when B button is pressed
    driver
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtVirtualTarget(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> FieldConstants.Hub.hubCenter));

    driver // Sohams way of SOTM using akit temp
        .leftBumper()
        .whileTrue(
            Commands.run(
                    () -> {
                      LaunchingParameters parms =
                          LauncherCalculator.getInstance()
                              .getParameters(
                                  drive::getPose, drive::getChassisSpeeds, drive::getRotation);
                      Logger.recordOutput("SOTM/Distance", parms.distance());
                      Logger.recordOutput(
                          "SOTM/Distance No Lookahead", parms.distanceNoLookahead());
                      Logger.recordOutput("SOTM/Drive Velocity", parms.driveVelocity());
                      Logger.recordOutput("SOTM/Flywheel Speed", parms.flywheelSpeed());
                      Logger.recordOutput("SOTM/Hood Angle", parms.hoodAngle());
                      Logger.recordOutput("SOTM/Is Valid", parms.isValid());
                      Logger.recordOutput("SOTM/Drive Angle", parms.driveAngle());
                      Logger.recordOutput(
                          "SOTM/Drive Angle No Lookahead", parms.driveAngleNoLookahead());
                      Logger.recordOutput("SOTM/Time of Flight", parms.timeOfFlight());

                      if (parms.isValid()) {
                        shooter.setVelocitySetpoint(RadiansPerSecond.of(parms.flywheelSpeed()));
                        hood.setAngle(Radians.of(parms.hoodAngle()));
                      }
                    })
                .alongWith(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driver.getLeftY() * 0.5,
                        () -> -driver.getLeftX() * 0.5,
                        () ->
                            LauncherCalculator.getInstance()
                                .getParameters(
                                    drive::getPose, drive::getChassisSpeeds, drive::getRotation)
                                .driveAngle())));

    driver
        .y()
        .whileTrue(
            Commands.run(
                () -> {
                  var chassisSpeeds = drive.getChassisSpeeds();
                  double speedMps =
                      Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
                  boolean isMoving = speedMps >= ShooterConstants.Targeting.movingSpeedThresholdMps;

                  ShootingCalculator.ShootingSolution solution =
                      isMoving
                          ? ShootingCalculator.calculateMovingSolution(
                              drive.getPose(),
                              chassisSpeeds,
                              Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg),
                              Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg),
                              ShooterConstants.Targeting.minRpm,
                              ShooterConstants.Targeting.maxRpm,
                              ShooterConstants.Targeting.movingRpmChangeWeight,
                              ShooterConstants.Targeting.movingHoodChangeWeight)
                          : ShootingCalculator.calculateStationarySolution(
                              drive.getPose(),
                              Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg),
                              Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg),
                              ShooterConstants.Targeting.stationaryRpm);

                  hood.setAngle(Degrees.of(Math.toDegrees(solution.hoodAngleRad)));
                  shooter.setVelocitySetpoint(
                      RadiansPerSecond.of(
                          Units.rotationsPerMinuteToRadiansPerSecond(solution.flywheelRpm)));

                  Logger.recordOutput(
                      "Superstructure/Target/HoodAngleDeg", Math.toDegrees(solution.hoodAngleRad));
                  Logger.recordOutput("Superstructure/Target/FlywheelRpm", solution.flywheelRpm);
                  Logger.recordOutput(
                      "Superstructure/Target/DistanceMeters", solution.distanceMeters);
                  Logger.recordOutput("Superstructure/Target/Valid", solution.valid);
                  Logger.recordOutput("Superstructure/Target/SpeedMps", speedMps);
                  Logger.recordOutput("Superstructure/Target/IsMoving", isMoving);
                }));
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
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

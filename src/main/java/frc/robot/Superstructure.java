package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBuilder;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.util.shooter.LauncherCalculator;
import frc.robot.util.shooter.LauncherCalculator.LaunchingParameters;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  public static class ControllerLayout {
    public static Trigger scoreRequest = new Trigger(() -> false);
    public static Trigger intakeRequest = new Trigger(() -> false);
    public static Trigger cancelRequest = new Trigger(() -> false);
    public static Trigger cancel = new Trigger(() -> false);
    public static Trigger disableTargeting = new Trigger(() -> false);
    public static Trigger passingRequest = new Trigger(() -> false);
    public static Trigger climbRequest = new Trigger(() -> false);
    public static DoubleSupplier joystickX = () -> (0.0);
    public static DoubleSupplier joystickY = () -> (0.0);
  }

  public enum State {
    idle,
    holding,
    shoot,
    pass,
    intake,
    climb,
    climbscore,
  }

  private HashMap<Trigger, State> stateRequests = new HashMap<Trigger, State>();
  private HashMap<State, Trigger> stateTriggers = new HashMap<State, Trigger>();

  @AutoLogOutput(key = "Superstructure/State/CurrentState")
  private State state = State.idle;

  @AutoLogOutput(key = "Superstructure/State/PreviousState")
  private State previousState = State.idle;

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Hood hood;
  private final Climb climb;
  private final AutoBuilder autobuilder;
  private final Supplier<Pose2d> drivePose;
  private final Supplier<ChassisSpeeds> robotRelativeSpeeds;
  private final Supplier<Rotation2d> driveHeading;

  @AutoLogOutput(key = "Superstructure/Target/Use Targetting")
  private boolean useTargeting = true;

  private boolean redStart = false;

  private Timer timer = new Timer();

  public Superstructure(
      final Drive drive,
      final Intake intake,
      final Shooter shooter,
      final Indexer indexer,
      final Hood hood,
      final Climb climb,
      final AutoBuilder autobuilder,
      final Supplier<Pose2d> drivePose,
      final Supplier<ChassisSpeeds> robotRelativeSpeeds,
      final Supplier<Rotation2d> headingSupplier) {
    // Assigning subsystems
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hood = hood;
    this.climb = climb;
    this.autobuilder = autobuilder;
    this.drivePose = drivePose;
    this.robotRelativeSpeeds = robotRelativeSpeeds;
    this.driveHeading = headingSupplier;

    for (State state : State.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    stateRequests.put(
        ControllerLayout.intakeRequest.and(stateTriggers.get(State.idle)), State.holding);
    stateRequests.put(
        ControllerLayout.intakeRequest.negate().and(stateTriggers.get(State.intake)),
        State.holding);
    stateRequests.put(
        ControllerLayout.passingRequest.negate().and(stateTriggers.get(State.pass)), State.holding);
    stateRequests.put(
        stateTriggers
            .get(State.shoot)
            .and(() -> !FieldConstants.LinesVertical.inAllianceZone(drive.getPose())),
        State.holding); // Save This one for later

    stateRequests.put(
        stateTriggers
            .get(State.holding)
            .and(() -> FieldConstants.LinesVertical.inAllianceZone(drive.getPose())),
        State.shoot); // Save This one for later
    stateRequests.put(
        ControllerLayout.intakeRequest.and(stateTriggers.get(State.holding)), State.intake);
    stateRequests.put(ControllerLayout.climbRequest, State.climb);
    stateRequests.put(
        ControllerLayout.scoreRequest.and(stateTriggers.get(State.climb)), State.climbscore);
    stateRequests.put(
        ControllerLayout.passingRequest.and(stateTriggers.get(State.holding)), State.pass);

    // State Trigger stuff here
    for (Trigger key : stateRequests.keySet()) {
      key.onTrue(setState(stateRequests.get(key)));
    }

    this.setupIdle();
    this.setupIntake();
    this.setupShoot();
    this.setupTarget();
    this.setupPass();
    this.setupClimb();
    this.setupCancel();

    timer.start();
  }

  private void setupCancel() {
    ControllerLayout.cancelRequest.onTrue(setState(State.holding));
    ControllerLayout.cancelRequest.multiPress(2, 1.5).onTrue(setState(State.idle));
  }

  private void setupIdle() {
    stateTriggers
        .get(State.idle)
        .whileTrue(
            Commands.parallel(
                indexer.setVoltage(Volts.of(0.0)),
                intake.setVoltage(Volts.of(0.0)),
                Commands.runOnce(() -> shooter.setVelocitySetpoint(RadiansPerSecond.of(0.0))),
                intake.setPosition(IntakeConstants.setpoints.stowed)));
  }

  private void setupIntake() {
    stateTriggers.get(State.holding).onTrue(intake.setPosition(IntakeConstants.setpoints.deployed));
    stateTriggers
        .get(State.holding)
        .whileTrue(
            Commands.parallel(
                Commands.runOnce(() -> shooter.setVoltage(0.0)),
                indexer.setVoltage(Volts.of(0.0)),
                intake.setVoltage(Volts.of(0.0)),
                shooter.runFeederVoltage(0.0)));
    stateTriggers
        .get(State.intake)
        .whileTrue(
            Commands.parallel(
                intake.setVoltage(Volts.of(12.0)),
                indexer.setVoltage(IndexerConstants.Setpoints.intake)));
  }

  private void setupShoot() {
    stateTriggers
        .get(State.shoot)
        .and(ControllerLayout.scoreRequest)
        .whileTrue(
            Commands.parallel(
                shooter.runFeederVoltage(8.0),
                indexer.setVoltage(IndexerConstants.Setpoints.feed)));
  }

  private void setupTarget() {
    stateTriggers
        .get(State.shoot)
        .and(() -> (FieldConstants.LinesVertical.inAllianceZone(drivePose.get())))
        .and(this::useTargeting)
        .whileTrue(
            Commands.run(
                () -> {
                  LaunchingParameters parms =
                      LauncherCalculator.getInstance()
                          .getParameters(drivePose, robotRelativeSpeeds, driveHeading);
                  if (parms.isValid()) {
                    shooter.setVelocitySetpoint(RadiansPerSecond.of(parms.flywheelSpeed()));
                    hood.setAngle(Radians.of(parms.hoodAngle()));
                  }
                }));
    // Commands.run(
    //     () -> {
    //       var chassisSpeeds = drive.getChassisSpeeds();
    //       double speedMps = Math.hypot(chassisSpeeds.vxMetersPerSecond,
    // chassisSpeeds.vyMetersPerSecond);
    //       boolean isMoving = speedMps >= ShooterConstants.Targeting.movingSpeedThresholdMps;

    //       ShootingCalculator.ShootingSolution solution = isMoving
    //           ? ShootingCalculator.calculateMovingSolution(
    //               drive.getPose(),
    //               chassisSpeeds,
    //               Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg),
    //               Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg),
    //               ShooterConstants.Targeting.minRpm,
    //               ShooterConstants.Targeting.maxRpm,
    //               ShooterConstants.Targeting.movingRpmChangeWeight,
    //               ShooterConstants.Targeting.movingHoodChangeWeight)
    //           : ShootingCalculator.calculateStationarySolution(
    //               drive.getPose(),
    //               Units.degreesToRadians(HoodConstants.Targeting.minAngleDeg),
    //               Units.degreesToRadians(HoodConstants.Targeting.maxAngleDeg),
    //               ShooterConstants.Targeting.stationaryRpm);

    //       hood.setAngle(Degrees.of(Math.toDegrees(solution.hoodAngleRad)));
    //       shooter.setVelocitySetpoint(
    //           RadiansPerSecond.of(
    //               Units.rotationsPerMinuteToRadiansPerSecond(solution.flywheelRpm)));

    //       Logger.recordOutput(
    //           "Superstructure/Target/HoodAngleDeg", Math.toDegrees(solution.hoodAngleRad));
    //       Logger.recordOutput("Superstructure/Target/FlywheelRpm", solution.flywheelRpm);
    //       Logger.recordOutput(
    //           "Superstructure/Target/DistanceMeters", solution.distanceMeters);
    //       Logger.recordOutput("Superstructure/Target/Valid", solution.valid);
    //       Logger.recordOutput("Superstructure/Target/SpeedMps", speedMps);
    //       Logger.recordOutput("Superstructure/Target/IsMoving", isMoving);
    //     }));

    stateTriggers
        .get(State.shoot)
        .and(ControllerLayout.disableTargeting)
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.useTargeting = !this.useTargeting;
                }));

    stateTriggers
        .get(State.shoot)
        .and(() -> (FieldConstants.LinesVertical.inAllianceZone(drivePose.get())))
        .and(() -> !useTargeting)
        .whileTrue(
            Commands.run(
                () -> {
                  LaunchingParameters parms =
                      LauncherCalculator.getInstance()
                          .getParameters(drivePose, robotRelativeSpeeds, driveHeading);
                  if (parms.isValid()) {
                    shooter.setVelocitySetpoint(RadiansPerSecond.of(parms.flywheelSpeed()));
                    hood.setAngle(Radians.of(parms.hoodAngle()));
                  }
                  Logger.recordOutput("SOTM/Flywheel Speed", parms.flywheelSpeed());
                  Logger.recordOutput("SOTM/Hood Angle", parms.hoodAngle());
                  Logger.recordOutput("SOTM/TOF", parms.timeOfFlight());
                }));
  }

  private void setupPass() {
    stateTriggers
        .get(State.pass)
        .whileTrue(
            Commands.run(
                () -> {
                  shooter.setVelocitySetpoint(RadiansPerSecond.of(300.0));
                  hood.setAngle(Radians.of(HoodConstants.Setpoints.passAngle.getRadians()));
                }));

    stateTriggers
        .get(State.pass)
        .and(ControllerLayout.scoreRequest)
        .and(() -> (!FieldConstants.LinesVertical.inAllianceZone(drivePose.get())))
        .whileTrue(
            Commands.parallel(
                indexer.setVoltage(IndexerConstants.Setpoints.feed),
                shooter.runFeederVoltage(12.0))); // Continue Targetting & Flywheel set speed.
  }

  private void setupClimb() {
    stateTriggers.get(State.climb).whileTrue(climb.setPosition(ClimbConstants.extendendHeight));

    stateTriggers
        .get(State.climbscore)
        .whileTrue(climb.setPosition(ClimbConstants.retractedHeight));
  }

  public Command setState(State newState) {
    return Commands.runOnce(
        () -> {
          previousState = state;
          state = newState;
        });
  }

  private boolean useTargeting() {
    return useTargeting;
  }

  @Override
  public void periodic() {
    // This method will only be used for logging and nothing else.
    for (State key : stateTriggers.keySet()) {
      Logger.recordOutput(
          "Superstructure/States/" + key.toString(), stateTriggers.get(key).getAsBoolean());
    }
    Logger.recordOutput("Superstructure/Layout/Cancel", ControllerLayout.cancelRequest);
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      redStart = gameData.charAt(0) == 'R';
    }
    Logger.recordOutput(
        "Superstructure/Hub Active",
        FieldConstants.Hub.isHubActive(redStart ? Alliance.Red : Alliance.Blue));
    autobuilder.updateField();
  }
}

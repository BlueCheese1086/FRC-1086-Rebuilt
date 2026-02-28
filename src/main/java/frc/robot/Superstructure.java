package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShotCalc;
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
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Hub;
// import frc.robot.util.shooter.LauncherCalculator;
// import frc.robot.util.shooter.LauncherCalculator.LaunchingParameters;
import frc.robot.util.PoseMath;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")
public class Superstructure extends SubsystemBase {

  public static class ControllerLayout {
    public static Trigger scoreRequest = new Trigger(() -> false);
    public static Trigger intakeRequest = new Trigger(() -> false);
    public static Trigger cancelRequest = new Trigger(() -> false);
    public static Trigger flushRequest = new Trigger(() -> false);
    public static Trigger disableTargeting = new Trigger(() -> false);
    public static Trigger passingRequest = new Trigger(() -> false);
    public static Trigger climbRequest = new Trigger(() -> false);
    public static Trigger agitate = new Trigger(() -> false);
    public static Trigger increaseRPM = new Trigger(() -> false);
    public static Trigger decreaseRPM = new Trigger(() -> false);
    public static DoubleSupplier joystickX = () -> (0.0);
    public static DoubleSupplier joystickY = () -> (0.0);
    public static Supplier<GenericHID> driverHid = () -> null;
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

  @SuppressWarnings("unused")
  private final Drive drive;

  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Hood hood;
  private final Climb climb;
  private final Supplier<Pose2d> drivePose;
  private final Supplier<ChassisSpeeds> robotRelativeSpeeds;
  private final Supplier<Rotation2d> driveHeading;

  @AutoLogOutput(key = "Superstructure/Target/Use Targetting")
  private boolean useTargeting = false;

  private boolean redStart = false;

  private Timer timer = new Timer();

  private double desiredRPM =
      0; // TODO update this to a decent RPM once shooting tests have been determined

  public Superstructure(
      final Drive drive,
      final Intake intake,
      final Shooter shooter,
      final Indexer indexer,
      final Hood hood,
      final Climb climb,
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

    Trigger enabled = new Trigger(DriverStation::isEnabled);
    enabled
        .and(() -> ControllerLayout.driverHid.get() != null)
        .whileTrue(rumbleBeforePeriodEnd(ControllerLayout.driverHid.get(), 4.0, 1.0));
  }

  // Driver awareness: rumble 4s before end of each period.
  // Put this in Superstructure so it always runs when the robot is enabled.
  /**
   * Rumbles a controller starting {@code secondsBeforeEnd} seconds before the end of the current
   * DriverStation period (teleop/auto).
   *
   * <p>Stops rumbling whenever the command ends and when the robot becomes disabled.
   */
  public static Command rumbleBeforePeriodEnd(
      GenericHID controller, double secondsBeforeEnd, double rumbleStrength) {
    return Commands.run(
            () -> {
              // If match time is not available, DriverStation.getMatchTime() returns -1.
              double matchTime = DriverStation.getMatchTime();
              boolean shouldRumble =
                  DriverStation.isEnabled() && matchTime > 0.0 && matchTime <= secondsBeforeEnd;

              double strength = shouldRumble ? rumbleStrength : 0.0;
              controller.setRumble(RumbleType.kLeftRumble, strength);
              controller.setRumble(RumbleType.kRightRumble, strength);
            })
        .finallyDo(
            () -> {
              controller.setRumble(RumbleType.kLeftRumble, 0.0);
              controller.setRumble(RumbleType.kRightRumble, 0.0);
            })
        .ignoringDisable(true);
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
                Commands.runOnce(() -> shooter.setVelocitySetpoint(RadiansPerSecond::zero))));

    stateTriggers.get(State.idle).onTrue(intake.setPosition(IntakeConstants.Setpoints.stowed));
    ControllerLayout.flushRequest.whileTrue(Commands.parallel());
  }

  private void setupIntake() {
    stateTriggers.get(State.holding).onTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));
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

    stateTriggers
        .get(State.intake)
        .whileTrue(
            DriveCommands.joystickDriveSyom(
                drive, ControllerLayout.joystickX, ControllerLayout.joystickY));

    // While intaking, override turning control so robot yaw faces direction of travel (SYOM).
    // This makes lining the intake up with balls much easier.
    stateTriggers
        .get(State.intake)
        .whileTrue(
            DriveCommands.joystickDriveSyom(
                drive, ControllerLayout.joystickX, ControllerLayout.joystickY));
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
        .and(ControllerLayout.agitate)
        .onTrue(intake.setPosition(IntakeConstants.Setpoints.agitate))
        .onFalse(intake.setPosition(IntakeConstants.Setpoints.deployed));

    stateTriggers
        .get(State.shoot)
        .and(ControllerLayout.disableTargeting)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                ControllerLayout.joystickX,
                ControllerLayout.joystickY,
                () -> {
                  return PoseMath.getOrientationToTarget(
                      drive.getPose(), AllianceFlipUtil.apply(FieldConstants.Hub.hubCenter));
                }));

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
        .whileTrue(
            Commands.parallel(
                shooter.setVelocity(
                    () -> {
                      return RPM.of(
                          ShotCalc.getShot(
                                  Meters.of(
                                      PoseMath.getDistanceToTarget(drive.getPose(), Hub.hubCenter)))
                              .shooterRPM);
                    })));
    // hood.setPosition(
    //     () -> {
    //       return ShotCalc.getShot(
    //               Meters.of(
    //                   PoseMath.getDistanceToTarget(drive.getPose(), Hub.hubCenter)))
    //           .hoodPosition;
  }
  ; // Why was shoot on the move stuff put in here. This is the worst place to
  // put the shoot on the move stuff.

  private void setupPass() {
    stateTriggers
        .get(State.pass)
        .and(ControllerLayout.agitate)
        .onTrue(intake.setPosition(IntakeConstants.Setpoints.agitate));

    stateTriggers
        .get(State.pass)
        .whileTrue(
            Commands.parallel(
                hood.setAngle(
                    HoodConstants.Setpoints
                        .passAngle), // TODO make this target center of alliance zone.
                shooter.setVelocity(() -> (RotationsPerSecond.of(300)))));

    stateTriggers
        .get(State.pass)
        .and(ControllerLayout.scoreRequest)
        .and(() -> (!FieldConstants.LinesVertical.inAllianceZone(drivePose.get())))
        .whileTrue(
            Commands.parallel(
                indexer.setVoltage(IndexerConstants.Setpoints.feed),
                shooter.runFeederVoltage(
                    ShooterConstants.FeederSetpoints.run.in(
                        Volts)))); // Continue Targetting & Flywheel set speed.
  }

  private void setupClimb() {
    stateTriggers.get(State.climb).onTrue(climb.setPosition(ClimbConstants.Setpoints.climbExtend));

    stateTriggers
        .get(State.climbscore)
        .onTrue(climb.setPosition(ClimbConstants.Setpoints.climbScore));
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
  }
}

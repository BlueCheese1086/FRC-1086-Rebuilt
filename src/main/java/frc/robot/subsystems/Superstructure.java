// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  public static class ControllerLayout {
    public static Trigger scoreRequest = new Trigger(() -> false);
    public static Trigger intakeRequest = new Trigger(() -> false);
    public static Trigger cancelRequest = new Trigger(() -> false);
    public static Trigger disableTargeting = new Trigger(() -> false);
    public static Trigger passingRequest = new Trigger(() -> false);
    public static Trigger climbRequest = new Trigger(() -> false);
    public static DoubleSupplier joystickX = () -> (0.0);
    public static DoubleSupplier joystickY = () -> (0.0);
  }

  public enum State { // Ideas
    idle,
    holding,
    target,
    score,
    intake,
    climb,
    climbscore,
    prepass,
    pass
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

  @AutoLogOutput(key = "Superstructure/Target/Use Targetting")
  private boolean useTargeting = true;

  public Superstructure(final Drive drive, final Intake intake, final Shooter shooter, final Indexer indexer,final Hood hood) {
    // Assigning subsystems
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hood = hood;


    for (State state : State.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    stateRequests.put(ControllerLayout.cancelRequest.and(stateTriggers.get(State.holding)), State.idle);
    stateRequests.put(ControllerLayout.intakeRequest.and(stateTriggers.get(State.idle)), State.holding);
    stateRequests.put(ControllerLayout.cancelRequest.and(stateTriggers.get(State.target)), State.holding);
    stateRequests.put(ControllerLayout.intakeRequest.negate().and(stateTriggers.get(State.intake)), State.holding);
    stateRequests.put(ControllerLayout.passingRequest.negate().and(stateTriggers.get(State.prepass)), State.holding);
    stateRequests.put(ControllerLayout.scoreRequest.negate().and(stateTriggers.get(State.score)), State.holding); // Save This one for later
    stateRequests.put(stateTriggers.get(State.holding).and(() -> {return FieldConstants.LinesVertical.inAllianceZone(drive.getPose());}), State.target); // Save This one for later  
    stateRequests.put(ControllerLayout.scoreRequest.and(stateTriggers.get(State.target)),State.score);
    stateRequests.put(ControllerLayout.climbRequest.and(stateTriggers.get(State.score).negate()),State.climb);
    stateRequests.put(ControllerLayout.scoreRequest.and(stateTriggers.get(State.climb)),State.climbscore);
    stateRequests.put(ControllerLayout.passingRequest.and(stateTriggers.get(State.holding)),State.prepass);
    stateRequests.put(ControllerLayout.scoreRequest.negate().and(stateTriggers.get(State.pass)),State.prepass);
    stateRequests.put(ControllerLayout.scoreRequest.and(stateTriggers.get(State.prepass)),State.pass);


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
  }

  private void setupIdle() {
    stateTriggers.get(State.idle).onTrue(Commands.parallel(intake.setPosition(IntakeConstants.setpoints.stowed))); // TODO: Soham add climb stuff with setpoints once done.
    stateTriggers.get(State.idle).whileTrue(
      Commands.parallel(
        shooter.setVoltage(0.0),
        indexer.setVoltage(Volts.of(0.0)),
        intake.setVoltage(Volts.of(0.0)),
        shooter.runFeederVoltage(0.0)
      ));
  }

  private void setupIntake() {
    stateTriggers.get(State.holding).onTrue(intake.setPosition(IntakeConstants.setpoints.deployed));
    stateTriggers.get(State.holding).whileTrue(
      Commands.parallel(
        shooter.setVoltage(0.0),
        indexer.setVoltage(Volts.of(0.0)),
        intake.setVoltage(Volts.of(0.0)),
        shooter.runFeederVoltage(0.0)
      ));
    stateTriggers.get(State.intake).whileTrue(Commands.parallel(intake.setVoltage(Volts.of(12.0)),indexer.setVoltage(IndexerConstants.Setpoints.intake)));
  }

  private void setupShoot() {
    stateTriggers.get(State.score).whileTrue(Commands.parallel(shooter.runFeederVoltage(12.0),indexer.setVoltage(IndexerConstants.Setpoints.feed)));
  }

  private void setupTarget() {
    stateTriggers.get(State.target).and(this::useTargeting).whileTrue(Commands.parallel()); // TODO: Soham setup your shoot on the move and velocity thing here.
    stateTriggers.get(State.target).and(ControllerLayout.disableTargeting).onTrue(Commands.runOnce(() -> {this.useTargeting = !this.useTargeting;}));
  }

  private void setupPass() {
    stateTriggers.get(State.prepass).whileTrue(Commands.parallel(DriveCommands.joystickDrive(drive, ControllerLayout.joystickX, ControllerLayout.joystickY, () -> {return AllianceFlipUtil.apply(Rotation2d.k180deg).getRadians();}))); // TODO: Soham add the flywheel speed calculator & hood calculator
    stateTriggers.get(State.pass).whileTrue(Commands.parallel(DriveCommands.joystickDrive(drive, ControllerLayout.joystickX, ControllerLayout.joystickY, () -> {return AllianceFlipUtil.apply(Rotation2d.k180deg).getRadians();}),indexer.setVoltage(IndexerConstants.Setpoints.feed),shooter.runFeederVoltage(12.0))); // Continue Targetting & Flywheel set speed.
  }

  private void setupClimb() {
    stateTriggers.get(State.climb).onTrue(Commands.none()); // TODO: Soham add climb stuff with setpoints once done.
    stateTriggers.get(State.climbscore).onTrue(Commands.none());
  }

  private Command setState(State newState) {
    return Commands.run(
            () -> {
              state = newState;
            })
        .withTimeout(0.01);
  }

  private boolean useTargeting() {
    return useTargeting;
  }

  @Override
  public void periodic() {
    // This method will only be used for logging and nothing else.
  }
}

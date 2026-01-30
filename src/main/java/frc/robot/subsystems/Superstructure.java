// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  public enum State { // Ideas
    Intake,
    Shoot,
    PreShoot,
    Pass,
    Climb,
    Idle
  }

  private HashMap<State, Trigger> stateRequests = new HashMap<State, Trigger>();
  private HashMap<State, Trigger> stateTriggers = new HashMap<State, Trigger>();

  @AutoLogOutput(key = "RobotState/CurrentState")
  private State state = State.Idle;

  @AutoLogOutput(key = "RobotState/PreviousState")
  private State previousState = State.Idle;

  public Superstructure(final Drive drive) {
    for (State state : State.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    stateRequests.put(State.Intake, null);
    stateRequests.put(State.Shoot, null);
    stateRequests.put(State.PreShoot, null);
    stateRequests.put(State.Pass, null);
    stateRequests.put(State.Climb, null);
    stateRequests.put(State.Idle, null);

    // State Trigger stuff here
    for (State state : State.values()) {
      stateRequests.get(state).onTrue(setState(state));
    }

    this.setupIdle();
    this.setupIntake();
    this.setupShoot();
    this.setupPreShoot();
    this.setupPass();
    this.setupClimb();
  }

  private void setupIdle() {}

  private void setupIntake() {}

  private void setupShoot() {}

  private void setupPreShoot() {}

  private void setupPass() {}

  private void setupClimb() {}

  private Command setState(State newState) {
    return Commands.run(
            () -> {
              state = newState;
            })
        .withTimeout(0.01);
  }

  @Override
  public void periodic() {
    // This method will only be used for logging and nothing else.
  }
}

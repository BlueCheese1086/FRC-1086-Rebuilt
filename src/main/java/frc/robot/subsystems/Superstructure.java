// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;

public class Superstructure extends SubsystemBase {
  
  public enum State { // Ideas
    intake,
    travel,
    target,
    shoot,
    pass,
    climb,
    disabled,
    idle
  }

  @AutoLogOutput( key = "Superstructure/Current State")
  private State currentState = State.disabled;
  private HashMap<State,Trigger> stateMap = new HashMap<State,Trigger>();
  /** Creates a new Superstructure. */
  public Superstructure(final Drive drive) {
    for (State state : State.values()) {
      stateMap.put(state, new Trigger(() -> {return currentState == state;}));
    }
  }

  // Logging Only!
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

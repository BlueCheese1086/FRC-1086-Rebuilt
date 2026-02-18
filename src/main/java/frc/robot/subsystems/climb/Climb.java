// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final ClimbIO io;

  public Climb(ClimbIO io) {
    this.io = io;
    io.resetEncoder();
    this.inputs = new ClimbIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public Command setPosition(double position) {
    return this.run(
        () -> {
          inputs.targetPosition = position;
          io.setPosition(position);
        });
  }

  public Command setPositionDynamic(DoubleSupplier position) {
    return this.run(
        () -> {
          inputs.targetPosition = position.getAsDouble();
          io.setPosition(position.getAsDouble());
        });
  }

  public Command resetEncoder() {
    return Commands.runOnce(() -> io.resetEncoder()).ignoringDisable(true);
  }
}

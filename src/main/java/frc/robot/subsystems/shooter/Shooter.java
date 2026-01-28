// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterInputs;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterInputsAutoLogged();
  }

  public Command setRPM(double rpm) {
    return this.run(() -> {
      io.setRPM(rpm);
    });
  }

  public Command setVolts(double volts) {
    return this.run(() -> {
      io.setVolts(volts);
    }).finallyDo(() -> {
      io.setVolts(0.0);
    });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}

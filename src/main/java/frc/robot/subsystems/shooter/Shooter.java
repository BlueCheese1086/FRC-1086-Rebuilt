// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterInputsAutoLogged();
  }

  public Command setVelocity(AngularVelocity vel) {
    return this.run(() -> {
      io.setVelocity(vel);
    }).finallyDo(() -> {
      io.setVelocity(RadiansPerSecond.zero());
    });
  }

  public Command setVoltage(Voltage volts) {
    return this.run(() -> {
      io.setVoltage(volts);
    }).finallyDo(() -> {
      io.setVoltage(Volts.zero());
    });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/leftVelocity", inputs.leftVelocity);
    Logger.recordOutput("Shooter/middleVelocity", inputs.middleVelocity);
    Logger.recordOutput("Shooter/rightVelocity", inputs.rightVelocity);
  }
}

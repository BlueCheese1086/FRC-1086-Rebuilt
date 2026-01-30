// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private ShooterInputsAutoLogged[] inputs;
  private ShooterIO[] io;
  private FeederIO feederIO;
  private FeederIOInputsAutoLogged feederIOInputsAutoLogged;

  public Shooter( FeederIO feederIO, ShooterIO... io){
    this.io = io;
    this.feederIO = feederIO;
    this.feederIOInputsAutoLogged = new FeederIOInputsAutoLogged();
    inputs = new ShooterInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ShooterInputsAutoLogged();
    }
  }

  public Command setVelocity(AngularVelocity radPerSec) {
    return this.runOnce(() -> {
      for (int i = 0; i < io.length; i++) {
        io[i].setVelocity(radPerSec);
      }
    });
  }

  public Command runFeederVoltage(double volts) {
    return Commands.run(()-> feederIO.setFeedVoltage(volts), this);
  }

  public Command setVoltage(double volts) {
    return this.run(() -> {
      for (int i = 0; i < io.length; i++) {
        io[i].setVoltage(volts);
      }
    }).finallyDo(() -> {
      this.stopAll();
    });
  }

  private void stopAll() {
    for (int i = 0; i < io.length; i++) {
      io[i].setVoltage(0.0);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Shooter" + i, inputs[i]);
    }
    feederIO.updateInputs(feederIOInputsAutoLogged);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterInputsAutoLogged[] inputs;
  private ShooterIO[] io;
  private FeederIO feederIO;
  private FeederIOInputsAutoLogged feederIOInputsAutoLogged;

  public Shooter(FeederIO feederIO, ShooterIO... io) {
    this.io = io;
    this.feederIO = feederIO;
    this.feederIOInputsAutoLogged = new FeederIOInputsAutoLogged();
    inputs = new ShooterInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ShooterInputsAutoLogged();
    }
  }

  public Command setVelocity(Supplier<AngularVelocity> radPerSec) {
    return this.run(
        () -> {
          for (int i = 0; i < io.length; i++) {
            io[i].setVelocity(radPerSec.get());
          }
        }).finallyDo(() -> {
          for (int i=0; i<io.length; i++) {
            io[i].setVoltage(0.0);
          }
        });
  }

  public void setVelocitySetpoint(AngularVelocity radPerSec) {
    for (int i = 0; i < io.length; i++) {
      io[i].setVelocity(radPerSec);
    }
  }

  public Command runFeederVoltage(double volts) {
    return Commands.run(() -> feederIO.setFeedVoltage(volts), this).finallyDo(() -> {feederIO.setFeedVoltage(0.0);});
  }

  public void setVoltage(double volts) {
    for (int i = 0; i < io.length; i++) {
      io[i].setVoltage(volts);
    }
  }

  public void stopAll() {
    for (int i = 0; i < io.length; i++) {
      io[i].setVoltage(0.0);
    }
    feederIO.setFeedVoltage(0.0);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Shooter/Flywheel" + (i+1), inputs[i]);
    }
    feederIO.updateInputs(feederIOInputsAutoLogged);
    Logger.processInputs("Shooter/Feeder", feederIOInputsAutoLogged);
  }

  public Command sysid(double timeout, int i, String string) {
    return Commands.sequence(
        this.getShooterSysIdQuasistatic(Direction.kForward, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        this.getShooterSysIdQuasistatic(Direction.kReverse, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        this.getShooterSysIdDynamic(Direction.kForward, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        this.getShooterSysIdDynamic(Direction.kReverse, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        Commands.runOnce(() -> io[i].setVoltage(0.0)));
  }

  public Command getShooterSysIdQuasistatic(Direction direction, int index, String name) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null),
            new SysIdRoutine.Mechanism(
                volts -> io[index].setVoltage(volts.in(Volts)),
                log -> {
                  log.motor(name)
                      .voltage(Volts.of(inputs[index].appliedVoltage))
                      .angularPosition(Rotations.of(inputs[index].positionRadPerSec))
                      .angularVelocity(RotationsPerSecond.of(inputs[index].velocity));
                },
                this))
        .quasistatic(direction);
  }

  public Command getShooterSysIdDynamic(Direction direction, int index, String name) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null),
            new SysIdRoutine.Mechanism(
                volts -> io[index].setVoltage(volts.in(Volts)),
                log -> {
                  log.motor(name)
                      .voltage(Volts.of(inputs[index].appliedVoltage))
                      .angularPosition(Rotations.of(inputs[index].positionRadPerSec))
                      .angularVelocity(RotationsPerSecond.of(inputs[index].velocity));
                },
                this))
        .dynamic(direction);
  }
}

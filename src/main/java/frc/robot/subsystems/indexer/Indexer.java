// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();
  private final BatteryLogger logger;

  public Indexer(IndexerIO io, BatteryLogger logger) {
    this.io = io;
    this.logger = logger;
  }

  public Command setVoltage(Voltage applied) {
    return this.run(
            () -> {
              io.setVoltage(applied);
            })
        .finallyDo(
            () -> {
              io.setVoltage(Volts.zero());
            });
  }

  public Command setCurrent(Current applied) {
    return this.run(
            () -> {
              io.setCurrent(applied);
            })
        .finallyDo(
            () -> {
              io.setVoltage(Volts.zero());
            });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    logger.reportCurrentUsage("Hopper/Track Motor", false, inputs.supply.in(Amps));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IndexerIOSim implements IndexerIO {
    private final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.04, 1.0), DCMotor.getKrakenX60Foc(1));
    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerInputs inputs) {
        sim.update(0.020);
        inputs.connected = true;
        inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
        inputs.voltage = Volts.of(sim.getInputVoltage());
    }

    @Override
    public void setVoltage(Voltage applied) {
        sim.setInputVoltage(applied.in(Volts));
    }
}

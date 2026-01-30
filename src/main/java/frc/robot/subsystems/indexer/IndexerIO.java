// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface IndexerIO {
    @AutoLog
    public class IndexerInputs {
        public boolean connected = false;
        public AngularVelocity velocity = RadiansPerSecond.zero();
        public Current supply = Amps.zero();
        public Current stator = Amps.zero();
        public Temperature temp = Celsius.zero();
        public Voltage voltage = Volts.zero();
    }
    public default void updateInputs(IndexerInputs inputs) {}
    public default void setVoltage(Voltage applied) {}
    public default void setCurrent(Current applied) {}   
}

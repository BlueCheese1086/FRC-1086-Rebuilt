// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    public boolean hopperConnected = false;
    public boolean hopperAlive = false;
    public AngularVelocity hopperVelocity = RadiansPerSecond.zero();
    public Voltage hopperVoltage = Volts.zero();
    public Temperature hopperTemperature = Celsius.zero();
    public Current hopperSupplyCurrent = Amps.zero();
    public Current hopperStatorCurrent = Amps.zero();
  }

  public default void updateInputs(IndexerInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}
}

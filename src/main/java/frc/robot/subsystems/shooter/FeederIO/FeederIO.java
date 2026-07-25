// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.FeederIO;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FeederIO {
  @AutoLog
  public class FeederIOInputs {
    public boolean feederConnected = false;
    public boolean feederAlive = false;
    public AngularVelocity feederVelocity = RadiansPerSecond.zero();
    public Voltage feederVoltage = Volts.zero();
    public Temperature feederTemperature = Celsius.zero();
    public Current feederSupplyCurrent = Amps.zero();
    public Current feederStatorCurrent = Amps.zero();
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeedVoltage(double voltage) {}
}

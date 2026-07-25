// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public class ShooterInputs {
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public Angle position = Radians.zero();
    public boolean upperLeftConnected = false;
    public boolean upperLeftAlive = false;
    public Voltage upperLeftVoltage = Volts.of(0);
    public Temperature upperLeftTemperature = Celsius.zero();
    public Current upperLeftSupply = Amps.zero();
    public Current upperLeftStator = Amps.zero();

    public boolean lowerLeftConnected = false;
    public boolean lowerLeftAlive = false;
    public Voltage lowerLeftVoltage = Volts.of(0);
    public Temperature lowerLeftTemperature = Celsius.zero();
    public Current lowerLeftSupply = Amps.zero();
    public Current lowerLeftStator = Amps.zero();

    public boolean upperRightConnected = false;
    public boolean upperRightAlive = false;
    public Voltage upperRightVoltage = Volts.of(0);
    public Temperature upperRightTemperature = Celsius.zero();
    public Current upperRightSupply = Amps.zero();
    public Current upperRightStator = Amps.zero();

    public boolean lowerRightConnected = false;
    public boolean lowerRightAlive = false;
    public Voltage lowerRightVoltage = Volts.of(0);
    public Temperature lowerRightTemperature = Celsius.zero();
    public Current lowerRightSupply = Amps.zero();
    public Current lowerRightStator = Amps.zero();
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setVoltage(double volts) {}
}

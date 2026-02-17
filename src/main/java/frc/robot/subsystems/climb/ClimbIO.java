// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double targetPosition = 0.0;
    public boolean motorConnected = false;
    public Voltage volts = Volts.zero();
    public Angle angle = Radians.zero();
    public Distance linearPosition = Meters.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public LinearVelocity linearVelocity = MetersPerSecond.zero();
    public Temperature temp = Celsius.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    public double climbPosition = 0.0;
  }

  public default void setPosition(double position) {}

  public default void updateInputs(ClimbIOInputsAutoLogged inputs) {}

  public default void resetEncoder() {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocity) {}
}

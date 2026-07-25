// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HoodIO {
  @AutoLog
  public class HoodInputs {
    public boolean connected = false;
    public boolean alive = false;
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.of(0);
    public Temperature temperature = Celsius.zero();
    public Current supply = Amps.zero();
    public Current stator = Amps.zero();
    public double positionDeg = 0.0;
  }

  public default void updateInputs(HoodInputs inputs) {}

  public default void setAngle(Angle angle) {}

  public default void setVoltage(Voltage voltage) {}

  public default void resetEncoder() {}

  public default void setBrakeMode(boolean brake) {}

  public default void updatePID(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
public interface IntakeIO {
  @AutoLog
  public class IntakeInputs {
    public Angle pivotPosition = Radians.zero();
    public double pivotPositionDeg = 0.0;
    public AngularVelocity rollerVelocity = RadiansPerSecond.zero();

    public boolean pivotConnected = false;
    public boolean pivotAlive = false;
    public AngularVelocity pivotVelocity = RadiansPerSecond.zero();
    public Voltage pivotVoltage = Volts.zero();
    public Temperature pivotTemperature = Celsius.zero();
    public Current pivotSupply = Amps.zero();
    public Current pivotStator = Amps.zero();

    public boolean leftRollerConnected = false;
    public boolean leftRollerAlive = false;
    public AngularVelocity leftRollerVelocity = RadiansPerSecond.zero();
    public Voltage leftRollerVoltage = Volts.zero();
    public Temperature leftRollerTemperature = Celsius.zero();
    public Current leftRollerSupply = Amps.zero();
    public Current leftRollerStator = Amps.zero();
    public Current leftRollerTorque = Amps.zero();

    public boolean rightRollerConnected = false;
    public boolean rightRollerAlive = false;
    public AngularVelocity rightRollerVelocity = RadiansPerSecond.zero();
    public Voltage rightRollerVoltage = Volts.zero();
    public Temperature rightRollerTemperature = Celsius.zero();
    public Current rightRollerSupply = Amps.zero();
    public Current rightRollerStator = Amps.zero();
    public Current rightRollerTorque = Amps.zero();
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setRollerVoltage(Voltage output) {}

  public default void setRollerVelocity(AngularVelocity velocity) {}

  public default void setPivotPosition(Angle position) {}

  public default void setPivotVoltage(Voltage voltage) {}
}

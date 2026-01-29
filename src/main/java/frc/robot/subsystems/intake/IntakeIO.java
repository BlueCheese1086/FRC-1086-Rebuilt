// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public class IntakeInputs {
        public boolean pivotConnected = false;
        public Angle pivotAngle = Radians.zero();
        public AngularVelocity pivotVelocity = RadiansPerSecond.zero();
        public boolean nearSetpoint = false;
        public Current pivotSupply = Amps.zero();
        public Current pivotStator = Amps.zero();
        public Temperature pivotTemp = Celsius.zero();
        public Voltage pivotAppliedVoltage = Volts.zero();

        public boolean rollerConnected = false;
        public AngularVelocity rollerVelocity = RadiansPerSecond.zero();
        public Current rollerSupply = Amps.zero();
        public Current rollerStator = Amps.zero();
        public Temperature rollerTemp = Celsius.zero();
        public Voltage rollerAppliedVoltage = Volts.zero();
    }
    public default void updateInputs(IntakeInputs inputs) {}
    public default void setPosition(Angle angle) {}
    public default void setCurrent(Current desired) {}
    public default void setVoltage(Voltage applied) {}
    public default void setPivotVoltage(Voltage applied) {}
}

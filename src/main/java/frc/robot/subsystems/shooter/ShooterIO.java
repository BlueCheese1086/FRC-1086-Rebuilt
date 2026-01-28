// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public class ShooterInputs {
        public AngularVelocity leftVelocity, middleVelocity, rightVelocity = RadiansPerSecond.zero();
    }

    public default void updateInputs(ShooterInputs inputs) {}
    public default void setVelocity(AngularVelocity vel) {}
    public default void setVoltage(Voltage volts) {}
}

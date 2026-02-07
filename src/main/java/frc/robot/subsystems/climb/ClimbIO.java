// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.climb.ClimbConstants.Position;

/** Add your docs here. */
public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
        public double voltage = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
        public double position = 0.0;
        public double velocity = 0.0;
        public double temperature = 0.0;
        public boolean isConnected = false;
        public double setpoint = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}
    public default void setVoltage(double volts) {}
    public default void setPosition(Position position) {}
} 

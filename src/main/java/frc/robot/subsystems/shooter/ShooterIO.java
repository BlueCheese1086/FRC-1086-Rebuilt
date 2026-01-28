// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public class ShooterInputs {
        public double leftCurrentRPM;
        public double middleCurrentRPM;
        public double rightCurrentRPM;
    }

    public default void updateInputs(ShooterInputs inputs) {}
    public default void setRPM(double rpm) {}
    public default void setVolts(double volts) {}
}

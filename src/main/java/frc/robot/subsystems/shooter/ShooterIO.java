// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterInputs {
    // Shooter
    public double velocity = 0.0;
    public double temp = 0.0;
    public double positionRadPerSec = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double appliedVoltage = 0.0;
    public double setpoint = 0.0;
    public boolean atSetpoint = false;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setVelocity(AngularVelocity radPerSecond) {}

  public default void setVoltage(double volts) {}
}

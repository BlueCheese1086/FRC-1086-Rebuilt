// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double targetPosition = 0.0;
    public boolean motorConnected = false;
    public double volts = 0.0;
    public Angle angle = Radians.zero();
    public double temp = 0.0;
    public double statorCurrent;
    public double supplyCurrent;
    public double climbPosition = 0.0;

    public double velocity = 0.0;
  }

  public default void setPosition(double position) {}

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void resetEncoder() {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocity) {}
}

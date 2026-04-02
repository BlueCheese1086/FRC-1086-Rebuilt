// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HoodIO {
  @AutoLog
  public class HoodInputs {
    public double setPosition;
    public Angle setAngle;
    public double targetPosition;

    public boolean leftAtSetpoint;
    public double leftPosition;
    public boolean rightAtSetpoint;
    public double rightPosition;

    public double hoodAngle;
    public double absoulteAngle;
  }

  public default void updateInputs(HoodInputs inputs) {}

  public default void setPosition(double position) {}

  public default boolean atSetpoint() {
    return false;
  }
}

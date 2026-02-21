// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class HoodIOSim implements HoodIO {
  private double prevDelta = 0.0;
  private double setpoint = 0.0;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodInputs inputs) {
    double deltaTime = Timer.getFPGATimestamp() - prevDelta;
    prevDelta = Timer.getFPGATimestamp();
    inputs.setPosition = setpoint;
    if (MathUtil.isNear(setpoint, inputs.leftPosition, 0.1)) {
      inputs.leftPosition = setpoint;
      inputs.rightPosition = setpoint;
    } else {
      inputs.leftPosition +=
          HoodConstants.Mechanical.kMaxServoSpeed.in(Millimeters.per(Second))
              * deltaTime
              * Math.signum(setpoint - inputs.leftPosition);
      inputs.rightPosition +=
          HoodConstants.Mechanical.kMaxServoSpeed.in(Millimeters.per(Second))
              * deltaTime
              * Math.signum(setpoint - inputs.rightPosition);
    }
  }

  @Override
  public void setPosition(double position) {
    setpoint = position * HoodConstants.Mechanical.kServoLength.in(Millimeters);
  }

  @Override
  public boolean atSetpoint() {
    return true;
  }
}

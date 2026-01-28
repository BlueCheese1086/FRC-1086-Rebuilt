// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

/** Add your docs here. */
public class HoodIOSim implements HoodIO {
    private double setpoint = 0.0;

    public HoodIOSim() {}

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.setPosition = setpoint;
        inputs.leftPosition = 0.0;
    }

    @Override
    public void setPosition(double position) {
        setpoint = position;
    }

    @Override
    public boolean atSetpoint() {return true;}
}

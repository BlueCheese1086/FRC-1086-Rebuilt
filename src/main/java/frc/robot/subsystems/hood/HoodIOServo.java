// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

/** Add your docs here. */
public class HoodIOServo implements HoodIO {
    private final Servo left;
    private final Servo right;
    
    private double prevDelta = 0.0;
    private double setpoint = 0.5;

    public HoodIOServo() {
        left = new Servo(RobotMap.Hood.left);
        right = new Servo(RobotMap.Hood.right);

        left.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        right.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    @Override
    public void setPosition(double position) {
        double clampedPosition = MathUtil.clamp(position,HoodConstants.Mechanical.minPosition,HoodConstants.Mechanical.maxPosition);
        left.set(clampedPosition*HoodConstants.Mechanical.scaledDist);
        right.set(clampedPosition*HoodConstants.Mechanical.scaledDist);
        setpoint = clampedPosition;
    }

    @Override
    public boolean atSetpoint() {
        return MathUtil.isNear(setpoint, left.get(), HoodConstants.Mechanical.kPositionTolerance) && MathUtil.isNear(setpoint, right.get(), HoodConstants.Mechanical.kPositionTolerance);
    }

    
    @Override
    public void updateInputs(HoodInputs inputs) {
        double deltaTime = Timer.getFPGATimestamp()-prevDelta;
        prevDelta = Timer.getFPGATimestamp();
        inputs.setPosition = setpoint;
        inputs.leftPosition += HoodConstants.Mechanical.kMaxServoSpeed.in(Millimeters.per(Second))*deltaTime * Math.signum(setpoint-inputs.leftPosition);
        inputs.rightPosition += HoodConstants.Mechanical.kMaxServoSpeed.in(Millimeters.per(Second))*deltaTime * Math.signum(setpoint-inputs.rightPosition);
        inputs.leftAtSetpoint = MathUtil.isNear(setpoint, inputs.leftPosition, HoodConstants.Mechanical.kPositionTolerance);
        inputs.rightAtSetpoint = MathUtil.isNear(setpoint, inputs.rightPosition, HoodConstants.Mechanical.kPositionTolerance);
    }
}

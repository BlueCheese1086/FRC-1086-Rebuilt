// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX left;
    private final TalonFX middle;
    private final TalonFX right;

    private final VelocityVoltage left_v;
    private final VelocityVoltage middle_v;
    private final VelocityVoltage right_v;

    public ShooterIOTalonFX() {
        left = new TalonFX(RobotMap.Shooter.left);
        middle = new TalonFX(RobotMap.Shooter.middle);
        right = new TalonFX(RobotMap.Shooter.right);

        left_v = new VelocityVoltage(0);
        middle_v = new VelocityVoltage(0);
        right_v = new VelocityVoltage(0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.PID.kP;
        config.Slot0.kI = ShooterConstants.PID.kI;
        config.Slot0.kD = ShooterConstants.PID.kD;
        config.Slot0.kV = ShooterConstants.PID.kV;

        left.getConfigurator().apply(config);
        middle.getConfigurator().apply(config);
        right.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.leftCurrentRPM = Units.radiansPerSecondToRotationsPerMinute(left.getVelocity().getValueAsDouble());
        inputs.middleCurrentRPM = Units.radiansPerSecondToRotationsPerMinute(middle.getVelocity().getValueAsDouble());
        inputs.rightCurrentRPM = Units.radiansPerSecondToRotationsPerMinute(right.getVelocity().getValueAsDouble());
    }

    @Override
    public void setRPM(double rpm) {
        double rps = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
        left.setControl(left_v.withVelocity(rps));
        middle.setControl(middle_v.withVelocity(rps));
        right.setControl(right_v.withVelocity(rps));
    }

    @Override
    public void setVolts(double volts) {
        left.setVoltage(volts);
        middle.setVoltage(volts);
        right.setVoltage(volts);
    }
}

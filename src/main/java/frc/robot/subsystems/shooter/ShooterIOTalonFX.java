// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX left, middle, right;
    private final VelocityVoltage left_v, middle_v, right_v;

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

        config.Slot0.kS = ShooterConstants.PID.kS;
        config.Slot0.kV = ShooterConstants.PID.kV;
        config.Slot0.kA = ShooterConstants.PID.kA;

        left.getConfigurator().apply(config);
        middle.getConfigurator().apply(config);
        right.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.leftVelocity = left.getVelocity().getValue();
        inputs.middleVelocity = middle.getVelocity().getValue();
        inputs.rightVelocity = right.getVelocity().getValue();
    }

    @Override
    public void setVelocity(AngularVelocity vel) {
        left.setControl(left_v.withVelocity(vel));
        middle.setControl(middle_v.withVelocity(vel));
        right.setControl(right_v.withVelocity(vel));
    }

    @Override
    public void setVoltage(Voltage volts) {
        left.setVoltage(volts.in(Volts));
        middle.setVoltage(volts.in(Volts));
        right.setVoltage(volts.in(Volts));
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX shooter;
    private final MotionMagicVelocityVoltage velocityVoltage;
    private final BangBangController bbController;

    // Status Signals
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<AngularAcceleration> acceleration;
    private StatusSignal<Current> statorCurrent;
    private StatusSignal<Current> supplyCurrent;
    private StatusSignal<Voltage> volts;
    private StatusSignal<Angle> position;
    private StatusSignal<Temperature> temp;

    private double setpoint = 0.0;


    public ShooterIOTalonFX(int id) {
        shooter = new TalonFX(id);
        bbController = new BangBangController();
        velocityVoltage = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = ShooterConstants.PID.kS;
        config.Slot0.kV = ShooterConstants.PID.kV;
        config.Slot0.kA = ShooterConstants.PID.kA;

        config.Audio.BeepOnBoot = true;
        config.MotionMagic.MotionMagicAcceleration = 0.0;
        config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.Feedback.SensorToMechanismRatio = Math.PI * 2;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        tryUntilOk(5, () -> shooter.getConfigurator().apply(config));

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                acceleration,
                position,
                statorCurrent,
                supplyCurrent,
                temp,
                velocity,
                volts);

        acceleration = shooter.getAcceleration();
        position = shooter.getPosition();
        statorCurrent = shooter.getStatorCurrent();
        supplyCurrent = shooter.getSupplyCurrent();
        temp = shooter.getDeviceTemp();
        velocity = shooter.getVelocity();
        volts = shooter.getMotorVoltage();

        shooter.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        BaseStatusSignal.refreshAll(
                acceleration,
                position,
                statorCurrent,
                supplyCurrent,
                temp,
                velocity,
                volts);

        inputs.velocity = velocity.getValueAsDouble();
        inputs.appliedVoltage = volts.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.temp = temp.getValueAsDouble();
        inputs.positionRadPerSec = position.getValueAsDouble();

        shooter.setVoltage(bbController.calculate(inputs.velocity));
    }

    @Override
    public void setVelocity(double vel) {
        this.setpoint = vel;
        shooter.setControl(velocityVoltage.withVelocity(vel));
    }

    @Override
    public void setVoltage(double volts) {
        shooter.setVoltage(volts);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.PID.kA;
import static frc.robot.subsystems.intake.IntakeConstants.PID.kS;
import static frc.robot.subsystems.intake.IntakeConstants.PID.kV;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotMap;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooter;
  private final BangBangController bbController;

  @SuppressWarnings("unused")
  private final VelocityVoltage velocityVoltage;

  private final MotionMagicVelocityVoltage motionMagic;

  // Status Signals
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<AngularAcceleration> acceleration;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Angle> position;
  private StatusSignal<Temperature> temp;

  private double setpoint = 0.0;
  private double bangBangVoltage = 0.0;

  public ShooterIOTalonFX(int id, boolean inverted) {
    shooter = new TalonFX(id, RobotMap.systemBus);
    bbController = new BangBangController(RadiansPerSecond.of(10.0).in(RotationsPerSecond)); // bang bang

    velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);
    motionMagic = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = ShooterConstants.Tuning.kS;
    config.Slot0.kV = ShooterConstants.Tuning.kV;
    config.Slot0.kA = ShooterConstants.Tuning.kA;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    config.Audio.BeepOnBoot = true;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80.0; // arbittury
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0; // arbittury

    config.MotionMagic.MotionMagicJerk = 0.0;
    config.MotionMagic.MotionMagicAcceleration = 500.0;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    tryUntilOk(5, () -> shooter.getConfigurator().apply(config));

    acceleration = shooter.getAcceleration();
    position = shooter.getPosition();
    statorCurrent = shooter.getStatorCurrent();
    supplyCurrent = shooter.getSupplyCurrent();
    temp = shooter.getDeviceTemp();
    velocity = shooter.getVelocity();
    volts = shooter.getMotorVoltage();

    velocity.setUpdateFrequency(250.0);
    acceleration.setUpdateFrequency(250.0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, acceleration, position, statorCurrent, supplyCurrent, temp, volts);
    shooter.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    BaseStatusSignal.refreshAll(
        acceleration, position, statorCurrent, supplyCurrent, temp, velocity, volts);

    inputs.velocity = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVoltage = volts.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
    inputs.positionRadPerSec = position.getValueAsDouble();
    inputs.setpoint = setpoint;
    inputs.atSetpoint = bbController.atSetpoint();
    this.bangBangVoltage = bbController.calculate(velocity.getValue().in(RotationsPerSecond));

    if (kV.hasChanged(hashCode())) {
      resetValues();
    }

    if (kS.hasChanged(hashCode())) {
      resetValues();
    }

    if (kA.hasChanged(hashCode())) {
      resetValues();
    }
  }

  private void resetValues() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKA(kA.getAsDouble());
    slot0Configs.withKS(kS.getAsDouble());
    slot0Configs.withKV(kV.getAsDouble());
    shooter.getConfigurator().apply(slot0Configs, 0.25);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    this.setpoint = velocity.in(RadiansPerSecond);
    bbController.setSetpoint(velocity.in(RotationsPerSecond));
    shooter.setControl(
        motionMagic
            .withVelocity(velocity)
            .withFeedForward(bangBangVoltage * RobotController.getBatteryVoltage()));
  }

  @Override
  public void setVoltage(double volts) {
    shooter.setVoltage(volts);
    bbController.setSetpoint(0.0);
    if (volts == 0) {
      shooter.stopMotor();
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.Tuning.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooter;

  private final VelocityTorqueCurrentFOC torqueVelocity;

  // Status Signals
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<AngularAcceleration> acceleration;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Angle> position;
  private StatusSignal<Temperature> temp;

  private double setpoint = 0.0;

  public ShooterIOTalonFX(int id, boolean inverted, Slot0Configs magicNums) {
    shooter = new TalonFX(id, RobotMap.systemBus);
    torqueVelocity = new VelocityTorqueCurrentFOC(0.0).withUseTimesync(true).withSlot(0);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = magicNums;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    config.Audio.BeepOnBoot = true;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120.0; // arbittury
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80.0; // arbittury
    config.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;

    tryUntilOk(5, () -> shooter.getConfigurator().apply(config));

    acceleration = shooter.getAcceleration();
    position = shooter.getPosition();
    statorCurrent = shooter.getStatorCurrent();
    supplyCurrent = shooter.getSupplyCurrent();
    temp = shooter.getDeviceTemp();
    velocity = shooter.getVelocity();
    volts = shooter.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, velocity, acceleration, supplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, acceleration, position, statorCurrent, temp, volts);
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
    inputs.acceleration = acceleration.getValueAsDouble();
    inputs.atSetpoint = MathUtil.isNear(setpoint, velocity.getValue().in(RadiansPerSecond), 25.0);

    LoggedTunableNumber.ifChanged(hashCode(), () -> resetValues(), leftkP, leftkv, leftks, leftka);
  }

  private void resetValues() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKP(leftkP.getAsDouble());
    slot0Configs.withKV(leftkv.getAsDouble());
    slot0Configs.withKA(leftka.getAsDouble());
    slot0Configs.withKS(leftks.getAsDouble());
    shooter.getConfigurator().apply(slot0Configs, 0.2);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    this.setpoint = velocity.in(RadiansPerSecond);
    shooter.setControl(torqueVelocity.withVelocity(velocity));
    if (velocity.in(RadiansPerSecond) == 0.0) {
      shooter.stopMotor();
    }
  }

  @Override
  public void setVoltage(double volts) {
    shooter.setVoltage(volts);
    if (volts == 0) {
      shooter.stopMotor();
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.Tuning.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooter;

  @SuppressWarnings("unused")
  private final VelocityVoltage velocityVoltage;

  // Status Signals
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<AngularAcceleration> acceleration;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Angle> position;
  private StatusSignal<Temperature> temp;

  private double setpoint = 0.0;

  public ShooterIOTalonFX(int id, boolean inverted) {
    shooter = new TalonFX(id, RobotMap.systemBus);
    // added .withUseTimesync(true) to velocity voltage, but not sure if it will cause issues with
    // the way we are using them, will test and remove if it does
    velocityVoltage =
        new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0).withUseTimesync(true);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = ks.getAsDouble();
    config.Slot0.kV = kv.getAsDouble();
    config.Slot0.kA = ka.getAsDouble();
    config.Slot0.kD = kd.getAsDouble();
    config.Slot0.kP = kP.getAsDouble();
    config.Slot0.kI = kI.getAsDouble();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    config.Audio.BeepOnBoot = true;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120.0; // arbittury
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80.0; // arbittury

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
    Logger.recordOutput("Robot Map/Shooter Pro", shooter.getIsProLicensed().getValueAsDouble());
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
    inputs.atSetpoint = MathUtil.isNear(setpoint, velocity.getValue().in(RadiansPerSecond), 30.0);

    if (kd.hasChanged(hashCode())) {
      resetValues();
    }

    if (kv.hasChanged(hashCode())) {
      resetValues();
    }

    if (kI.hasChanged(hashCode())) {
      resetValues();
    }

    if (ka.hasChanged(hashCode())) {
      resetValues();
    }

    if (ks.hasChanged(hashCode())) {
      resetValues();
    }

    if (kP.hasChanged(hashCode())) {
      resetValues();
    }
  }

  private void resetValues() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKP(kP.getAsDouble());
    slot0Configs.withKI(kI.getAsDouble());
    slot0Configs.withKD(kd.getAsDouble());
    slot0Configs.withKV(kv.getAsDouble());
    slot0Configs.withKA(ka.getAsDouble());
    slot0Configs.withKS(ks.getAsDouble());
    shooter.getConfigurator().apply(slot0Configs, 0.2);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    this.setpoint = velocity.in(RadiansPerSecond);
    shooter.setControl(velocityVoltage.withVelocity(velocity));
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

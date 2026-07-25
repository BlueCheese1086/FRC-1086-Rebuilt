// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class HoodIOTalonFX implements HoodIO {

  private final TalonFX talon;
  private final TalonFXConfiguration configuration = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Integer> version;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> stator;
  private final StatusSignal<Current> supply;

  private final VoltageOut voltageControl = new VoltageOut(Volts.zero());
  private final PositionVoltage positionControl = new PositionVoltage(Radians.zero());

  public HoodIOTalonFX() {
    talon = new TalonFX(RobotMap.hood, RobotMap.systemBus);

    configuration.CurrentLimits.StatorCurrentLimit = HoodConstants.CurrentLimits.maxStator.in(Amps);
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = HoodConstants.CurrentLimits.maxStator.in(Amps);
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    configuration.Voltage.PeakForwardVoltage = HoodConstants.VoltageLimits.maxVoltage.in(Volts);
    configuration.Voltage.PeakReverseVoltage = HoodConstants.VoltageLimits.minVoltage.in(Volts);
    configuration.Feedback.SensorToMechanismRatio = HoodConstants.Mechanical.gearing;
    configuration.MotorOutput.PeakForwardDutyCycle = 1.0;
    configuration.MotorOutput.PeakReverseDutyCycle = -1.0;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configuration.Slot0.kP = 200.0;
    configuration.Slot0.kI = HoodConstants.PID.kI.get();
    configuration.Slot0.kD = 0.0;
    configuration.Slot0.kS = 0.23046875;
    configuration.Slot0.kG = HoodConstants.PID.kG.get();
    configuration.Slot0.kV = 24;
    configuration.Slot0.kA = 1.7207000255584717;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(configuration, 0.25));
    this.position = talon.getPosition();
    this.velocity = talon.getVelocity();
    this.appliedVoltage = talon.getMotorVoltage();
    this.temperature = talon.getDeviceTemp();
    this.supply = talon.getSupplyCurrent();
    this.stator = talon.getStatorCurrent();
    this.version = talon.getVersion();
    // It is at 50 because any higher is useless.
    PhoenixUtil.tryUntilOk(5, () -> StatusSignal.setUpdateFrequencyForAll(50, position, velocity));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            StatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, temperature, stator, supply));
    PhoenixUtil.tryUntilOk(5, () -> StatusSignal.setUpdateFrequencyForAll(4, version));
    PhoenixUtil.tryUntilOk(5, () -> talon.optimizeBusUtilization());
    resetEncoder();
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    StatusSignal.refreshAll(
        position, velocity, appliedVoltage, temperature, stator, supply, version);
    inputs.connected = talon.isConnected();
    inputs.alive =
        StatusSignal.isAllGood(
            position, velocity, appliedVoltage, temperature, stator, supply, version);
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = appliedVoltage.getValue();
    inputs.temperature = temperature.getValue();
    inputs.stator = stator.getValue();
    inputs.supply = supply.getValue();
  }

  @Override
  public void setAngle(Angle angle) {
    talon.setControl(positionControl.withPosition(angle));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    talon.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void resetEncoder() {
    talon.setPosition(Radians.zero());
  }

  @Override
  public void setBrakeMode(boolean brake) {
    configuration.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(configuration, 0.25));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VoltageOut applyVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC applyCurrent = new TorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> stator;
  private final StatusSignal<Current> supply;
  private final StatusSignal<Temperature> temp;

  public IndexerIOTalonFX() {
    talon = new TalonFX(RobotMap.indexer, RobotMap.systemBus);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IndexerConstants.CurrentLimits.maxStator.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.CurrentLimits.maxSupply.in(Amps);

    config.Voltage.PeakForwardVoltage = IndexerConstants.VoltageLimits.peakForwardVoltage.in(Volts);
    config.Voltage.PeakReverseVoltage = IndexerConstants.VoltageLimits.peakReverseVoltage.in(Volts);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(15, () -> (talon.getConfigurator().apply(config)));

    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    stator = talon.getStatorCurrent();
    supply = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVoltage, stator, supply, temp);

    PhoenixUtil.tryUntilOk(5, () -> (talon.optimizeBusUtilization()));
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    StatusSignal.refreshAll(velocity, appliedVoltage, stator, supply, temp);
    inputs.connected = StatusSignal.isAllGood(velocity, appliedVoltage, stator, supply, temp);
    inputs.velocity = velocity.getValue();
    inputs.stator = stator.getValue();
    inputs.supply = supply.getValue();
    inputs.temp = temp.getValue();
    inputs.voltage = appliedVoltage.getValue();
  }

  public void setVoltage(Voltage applied) {
    talon.setControl(applyVoltage.withOutput(applied));

    if (applied.magnitude() == 0) {
      talon.stopMotor();
    }
  }

  public void setCurrent(Current applied) {
    talon.setControl(applyCurrent.withOutput(applied));
  }
}

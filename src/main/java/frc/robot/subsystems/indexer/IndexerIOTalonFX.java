// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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
  private final TalonFX hopperTalon;
  private TalonFXConfiguration hopperConfig = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> stator;
  private final StatusSignal<Current> supply;
  private final StatusSignal<Temperature> temp;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0.0);

  public IndexerIOTalonFX() {
    hopperTalon = new TalonFX(RobotMap.hopper, RobotMap.systemBus);

    hopperConfig.CurrentLimits.StatorCurrentLimit =
        (int) IndexerConstants.CurrentLimits.maxStator.in(Amps);
    hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hopperConfig.CurrentLimits.SupplyCurrentLimit =
        (int) IndexerConstants.CurrentLimits.maxSupply.in(Amps);
    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hopperConfig.Voltage.PeakForwardVoltage =
        (int) IndexerConstants.VoltageLimits.maxForward.in(Volts);
    hopperConfig.Voltage.PeakReverseVoltage =
        (int) IndexerConstants.VoltageLimits.maxReverse.in(Volts);

    hopperConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hopperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> hopperTalon.getConfigurator().apply(hopperConfig, 0.25));

    velocity = hopperTalon.getVelocity();
    appliedVoltage = hopperTalon.getMotorVoltage();
    stator = hopperTalon.getStatorCurrent();
    supply = hopperTalon.getSupplyCurrent();
    temp = hopperTalon.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, velocity, appliedVoltage, stator, supply, temp));
    PhoenixUtil.tryUntilOk(5, () -> hopperTalon.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    StatusCode status = BaseStatusSignal.refreshAll(velocity, appliedVoltage, stator, supply, temp);
    inputs.hopperConnected = hopperTalon.isConnected();
    inputs.hopperAlive = status == StatusCode.OK;
    inputs.hopperVelocity = velocity.getValue();
    inputs.hopperVoltage = appliedVoltage.getValue();
    inputs.hopperStatorCurrent = stator.getValue();
    inputs.hopperSupplyCurrent = supply.getValue();
    inputs.hopperTemperature = temp.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    hopperTalon.setControl(voltageOut.withOutput(voltage));
  }
}

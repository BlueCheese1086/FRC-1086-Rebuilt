// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.FeederIO;

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
public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederTalon;
  private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> stator;
  private final StatusSignal<Current> supply;
  private final StatusSignal<Temperature> temp;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0.0);

  public FeederIOTalonFX() {
    feederTalon = new TalonFX(RobotMap.feeder, RobotMap.systemBus);

    feederConfig.CurrentLimits.StatorCurrentLimit =
        (int) FeederConstants.CurrentLimits.maxStator.in(Amps);
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.CurrentLimits.SupplyCurrentLimit =
        (int) FeederConstants.CurrentLimits.maxSupply.in(Amps);
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    feederConfig.Voltage.PeakForwardVoltage =
        (int) FeederConstants.VoltageLimits.maxForward.in(Volts);
    feederConfig.Voltage.PeakReverseVoltage =
        (int) FeederConstants.VoltageLimits.maxReverse.in(Volts);

    feederConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> feederTalon.getConfigurator().apply(feederConfig, 0.25));

    velocity = feederTalon.getVelocity();
    appliedVoltage = feederTalon.getMotorVoltage();
    stator = feederTalon.getStatorCurrent();
    supply = feederTalon.getSupplyCurrent();
    temp = feederTalon.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, velocity, appliedVoltage, stator, supply, temp));
    PhoenixUtil.tryUntilOk(5, () -> feederTalon.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    StatusCode status = BaseStatusSignal.refreshAll(velocity, appliedVoltage, stator, supply, temp);
    inputs.feederConnected = feederTalon.isConnected();
    inputs.feederAlive = status == StatusCode.OK;
    inputs.feederVelocity = velocity.getValue();
    inputs.feederVoltage = appliedVoltage.getValue();
    inputs.feederStatorCurrent = stator.getValue();
    inputs.feederSupplyCurrent = supply.getValue();
    inputs.feederTemperature = temp.getValue();
  }

  @Override
  public void setFeedVoltage(double voltage) {
    feederTalon.setControl(voltageOut.withOutput(voltage));
  }
}

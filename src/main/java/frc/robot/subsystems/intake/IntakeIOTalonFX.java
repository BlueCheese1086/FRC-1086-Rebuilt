// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  private final VoltageOut applyVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC applyCurrent = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC pivotPosition = new PositionTorqueCurrentFOC(0.0);
  private final TalonFX pivot;
  private final TalonFX roller;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerSupply;
  private final StatusSignal<Current> rollerStator;
  private final StatusSignal<Temperature> rollerTemperature;

  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotSupply;
  private final StatusSignal<Current> pivotStator;
  private final StatusSignal<Temperature> pivotTemperature;

  public IntakeIOTalonFX() {
    pivot = new TalonFX(RobotMap.IntakeMap.pivot, RobotMap.systemBus);
    roller = new TalonFX(RobotMap.IntakeMap.roller, RobotMap.systemBus);

    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.CurrentLimits.maxStator.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CurrentLimits.maxSupply.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Voltage.PeakForwardVoltage = IntakeConstants.VoltageLimits.peakForwardVoltage.in(Volts);
    config.Voltage.PeakReverseVoltage = IntakeConstants.VoltageLimits.peakForwardVoltage.in(Volts);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> (roller.getConfigurator().apply(config, 5)));

    // TODO: PID STUFF
    config.Slot0.kP = IntakeConstants.PID.kP;
    config.Slot0.kI = IntakeConstants.PID.kI;
    config.Slot0.kD = IntakeConstants.PID.kD;
    config.Slot0.kG = IntakeConstants.PID.kG;
    config.Slot0.kS = IntakeConstants.PID.kS;
    config.Slot0.kV = IntakeConstants.PID.kV;
    config.Slot0.kA = IntakeConstants.PID.kA;

    config.Feedback.SensorToMechanismRatio = IntakeConstants.Mechanical.gearing;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    PhoenixUtil.tryUntilOk(5, () -> (pivot.getConfigurator().apply(config, 5)));

    rollerVelocity = roller.getVelocity();
    rollerVoltage = roller.getMotorVoltage();
    rollerSupply = roller.getSupplyCurrent();
    rollerStator = roller.getStatorCurrent();
    rollerTemperature = roller.getDeviceTemp();

    pivotAngle = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotSupply = pivot.getSupplyCurrent();
    pivotStator = pivot.getStatorCurrent();
    pivotTemperature = pivot.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        RobotMap.systemBus.isNetworkFD() ? 250.0 : 50.0, pivotAngle);
    StatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerVoltage,
        rollerSupply,
        rollerStator,
        rollerTemperature,
        rollerVelocity,
        pivotVelocity,
        pivotVoltage,
        pivotSupply,
        pivotStator,
        pivotTemperature);
    PhoenixUtil.tryUntilOk(5, () -> roller.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> pivot.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    StatusSignal.refreshAll(
        rollerVelocity,
        rollerVoltage,
        rollerSupply,
        rollerStator,
        rollerTemperature,
        pivotAngle,
        pivotVelocity,
        pivotVoltage,
        pivotSupply,
        pivotStator,
        pivotTemperature);
    inputs.rollerConnected =
        StatusSignal.isAllGood(
            rollerVelocity, rollerVoltage, rollerSupply, rollerStator, rollerTemperature);
    inputs.rollerVelocity = rollerVelocity.getValue();
    inputs.rollerAppliedVoltage = rollerVoltage.getValue();
    inputs.rollerStator = rollerStator.getValue();
    inputs.rollerSupply = rollerSupply.getValue();
    inputs.rollerTemp = rollerTemperature.getValue();

    inputs.pivotConnected =
        StatusSignal.isAllGood(
            pivotAngle, pivotVelocity, pivotVoltage, pivotSupply, pivotStator, pivotTemperature);
    inputs.pivotAngle = pivotAngle.getValue();
    inputs.pivotVelocity = pivotVelocity.getValue();
    inputs.pivotStator = pivotStator.getValue();
    inputs.pivotSupply = pivotSupply.getValue();
    inputs.pivotAppliedVoltage = pivotVoltage.getValue();
    inputs.pivotTemp = pivotTemperature.getValue();
  }

  @Override
  public void setPosition(Angle angle) {
    pivot.setControl(pivotPosition.withPosition(angle));
  }

  @Override
  public void setCurrent(Current desired) {
    roller.setControl(applyCurrent.withOutput(desired));
  }

  @Override
  public void setVoltage(Voltage applied) {
    roller.setControl(applyVoltage.withOutput(applied));
  }
}

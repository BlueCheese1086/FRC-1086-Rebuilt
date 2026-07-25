// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX pivot;
  private TalonFX leftRoller;
  private TalonFX rightRoller;

  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Integer> pivotVersion;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<Current> pivotStator;
  private final StatusSignal<Current> pivotSupply;

  private final StatusSignal<AngularVelocity> leftRollerVelocity;
  private final StatusSignal<Integer> leftRollerVersion;
  private final StatusSignal<Voltage> leftRollerVoltage;
  private final StatusSignal<Temperature> leftRollerTemperature;
  private final StatusSignal<Current> leftRollerStator;
  private final StatusSignal<Current> leftRollerSupply;

  private final StatusSignal<AngularVelocity> rightRollerVelocity;
  private final StatusSignal<Integer> rightRollerVersion;
  private final StatusSignal<Voltage> rightRollerVoltage;
  private final StatusSignal<Temperature> rightRollerTemperature;
  private final StatusSignal<Current> rightRollerStator;
  private final StatusSignal<Current> rightRollerSupply;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final PositionVoltage positionVoltage = new PositionVoltage(Radians.zero());
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(RadiansPerSecond.zero());
  private final TorqueCurrentFOC currentOutput = new TorqueCurrentFOC(0.0);

  private final VoltageOut pivotAppliedVoltage = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    pivot = new TalonFX(RobotMap.Intake.pivot, RobotMap.systemBus);
    leftRoller = new TalonFX(RobotMap.Intake.leftRoller, RobotMap.systemBus);
    rightRoller = new TalonFX(RobotMap.Intake.rightRoller, RobotMap.systemBus);

    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.Mechanical.pivotGearing;
    pivotConfig.MotorOutput.Inverted =
        IntakeConstants.Mechanical.pivotInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    pivotConfig.MotorOutput.PeakForwardDutyCycle = IntakeConstants.Pivot.Output.maxForward;
    pivotConfig.MotorOutput.PeakReverseDutyCycle = IntakeConstants.Pivot.Output.maxReverse;
    pivotConfig.Slot0.kP = IntakeConstants.Pivot.PID.kP.getAsDouble();
    pivotConfig.Slot0.kI = IntakeConstants.Pivot.PID.kI.getAsDouble();
    pivotConfig.Slot0.kD = IntakeConstants.Pivot.PID.kD.getAsDouble();
    pivotConfig.Slot0.kS = IntakeConstants.Pivot.PID.kS.getAsDouble();
    pivotConfig.Slot0.kG = IntakeConstants.Pivot.PID.kG.getAsDouble();
    pivotConfig.Slot0.kV = IntakeConstants.Pivot.PID.kV.getAsDouble();
    pivotConfig.Slot0.kA = IntakeConstants.Pivot.PID.kA.getAsDouble();
    pivotConfig.CurrentLimits.SupplyCurrentLimit =
        (int) IntakeConstants.Pivot.CurrentLimits.maxSupply.in(Amps);
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        (int) IntakeConstants.Pivot.CurrentLimits.maxStator.in(Amps);
    pivotConfig.Voltage.PeakForwardVoltage =
        IntakeConstants.Pivot.VoltageLimits.maxForward.in(Volts);
    pivotConfig.Voltage.PeakReverseVoltage =
        IntakeConstants.Pivot.VoltageLimits.maxReverse.in(Volts);
    pivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

    rollerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.Mechanical.rollerGearing;
    rollerConfig.MotorOutput.Inverted =
        IntakeConstants.Mechanical.rollerInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.PeakForwardDutyCycle = IntakeConstants.Roller.Output.maxForward;
    rollerConfig.MotorOutput.PeakReverseDutyCycle = IntakeConstants.Roller.Output.maxReverse;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        (int) IntakeConstants.Roller.CurrentLimits.maxSupply.in(Amps);
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit =
        (int) IntakeConstants.Roller.CurrentLimits.maxStator.in(Amps);
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.Voltage.PeakForwardVoltage =
        IntakeConstants.Roller.VoltageLimits.maxForward.in(Volts);
    rollerConfig.Voltage.PeakReverseVoltage =
        IntakeConstants.Roller.VoltageLimits.maxReverse.in(Volts);
    rollerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
    rollerConfig.Slot0.kP = IntakeConstants.Roller.PID.kP.getAsDouble();
    rollerConfig.Slot0.kI = IntakeConstants.Roller.PID.kI.getAsDouble();
    rollerConfig.Slot0.kD = IntakeConstants.Roller.PID.kD.getAsDouble();
    rollerConfig.Slot0.kS = IntakeConstants.Roller.PID.kS.getAsDouble();
    rollerConfig.Slot0.kG = IntakeConstants.Roller.PID.kG.getAsDouble();
    rollerConfig.Slot0.kV = IntakeConstants.Roller.PID.kV.getAsDouble();
    rollerConfig.Slot0.kA = IntakeConstants.Roller.PID.kA.getAsDouble();

    PhoenixUtil.tryUntilOk(5, () -> pivot.getConfigurator().apply(pivotConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> leftRoller.getConfigurator().apply(rollerConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> rightRoller.getConfigurator().apply(rollerConfig, 0.25));

    pivotPosition = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotTemperature = pivot.getDeviceTemp();
    pivotStator = pivot.getStatorCurrent();
    pivotSupply = pivot.getSupplyCurrent();
    pivotVersion = pivot.getVersion();

    leftRollerVersion = leftRoller.getVersion();
    leftRollerVelocity = leftRoller.getVelocity();
    leftRollerVoltage = leftRoller.getMotorVoltage();
    leftRollerTemperature = leftRoller.getDeviceTemp();
    leftRollerStator = leftRoller.getStatorCurrent();
    leftRollerSupply = leftRoller.getSupplyCurrent();

    rightRollerVersion = rightRoller.getVersion();
    rightRollerVelocity = rightRoller.getVelocity();
    rightRollerVoltage = rightRoller.getMotorVoltage();
    rightRollerTemperature = rightRoller.getDeviceTemp();
    rightRollerStator = rightRoller.getStatorCurrent();
    rightRollerSupply = rightRoller.getSupplyCurrent();

    rightRoller.setControl(new Follower(leftRoller.getDeviceID(), MotorAlignmentValue.Opposed));

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTemperature,
                pivotStator,
                pivotSupply,
                leftRollerVelocity,
                leftRollerVoltage,
                leftRollerTemperature,
                leftRollerStator,
                leftRollerSupply,
                rightRollerVelocity,
                rightRollerVoltage,
                rightRollerTemperature,
                rightRollerStator,
                rightRollerSupply));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                4.0, pivotVersion, leftRollerVersion, rightRollerVersion));
    PhoenixUtil.tryUntilOk(5, () -> pivot.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> leftRoller.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> rightRoller.optimizeBusUtilization());
    pivot.setPosition(Radians.zero());
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    StatusSignal.refreshAll(
        pivotPosition,
        pivotVelocity,
        pivotVoltage,
        pivotTemperature,
        pivotStator,
        pivotSupply,
        pivotVersion,
        leftRollerVelocity,
        leftRollerVoltage,
        leftRollerTemperature,
        leftRollerStator,
        leftRollerSupply,
        leftRollerVersion,
        rightRollerVelocity,
        rightRollerVoltage,
        rightRollerTemperature,
        rightRollerStator,
        rightRollerSupply,
        rightRollerVersion);
    inputs.pivotConnected = pivot.isConnected();
    inputs.pivotAlive =
        StatusSignal.isAllGood(
            pivotPosition,
            pivotVelocity,
            pivotVoltage,
            pivotTemperature,
            pivotStator,
            pivotSupply,
            pivotVersion);
    inputs.pivotPosition = pivotPosition.getValue();
    inputs.pivotVelocity = pivotVelocity.getValue();
    inputs.pivotVoltage = pivotVoltage.getValue();
    inputs.pivotTemperature = pivotTemperature.getValue();
    inputs.pivotStator = pivotStator.getValue();
    inputs.pivotSupply = pivotSupply.getValue();

    inputs.leftRollerConnected = leftRoller.isConnected();
    inputs.leftRollerAlive =
        StatusSignal.isAllGood(
            leftRollerVelocity,
            leftRollerVoltage,
            leftRollerTemperature,
            leftRollerStator,
            leftRollerSupply,
            leftRollerVersion);
    inputs.leftRollerVelocity = leftRollerVelocity.getValue();
    inputs.leftRollerVoltage = leftRollerVoltage.getValue();
    inputs.leftRollerTemperature = leftRollerTemperature.getValue();
    inputs.leftRollerStator = leftRollerStator.getValue();
    inputs.leftRollerSupply = leftRollerSupply.getValue();

    inputs.rightRollerConnected = rightRoller.isConnected();
    inputs.rightRollerAlive =
        StatusSignal.isAllGood(
            rightRollerVelocity,
            rightRollerVoltage,
            rightRollerTemperature,
            rightRollerStator,
            rightRollerSupply,
            rightRollerVersion);
    inputs.rightRollerVelocity = rightRollerVelocity.getValue();
    inputs.rightRollerVoltage = rightRollerVoltage.getValue();
    inputs.rightRollerTemperature = rightRollerTemperature.getValue();
    inputs.rightRollerStator = rightRollerStator.getValue();
    inputs.rightRollerSupply = rightRollerSupply.getValue();
  }

  @Override
  public void setRollerVoltage(Voltage output) {
    leftRoller.setControl(voltageOut.withOutput(output));
  }

  @Override
  public void setRollerVelocity(AngularVelocity velocity) {
    leftRoller.setControl(velocityVoltage.withVelocity(velocity));
  }

  @Override
  public void setPivotPosition(Angle position) {
    pivot.setControl(positionVoltage.withPosition(position));
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    pivot.setControl(pivotAppliedVoltage.withOutput(voltage));
  }
}

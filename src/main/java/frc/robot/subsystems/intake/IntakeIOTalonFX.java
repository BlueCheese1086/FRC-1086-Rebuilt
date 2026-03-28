// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.PID.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  private final VoltageOut applyVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut applyPivotVoltage = new VoltageOut(0.0);
  private final TorqueCurrentFOC applyCurrent = new TorqueCurrentFOC(0.0);
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final TalonFX pivot;
  private final TalonFX rollerLeft;
  private final TalonFX rollerRight;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerLeftVoltage;
  private final StatusSignal<Current> rollerLeftSupply;
  private final StatusSignal<Current> rollerLeftStator;
  private final StatusSignal<Voltage> rollerRightVoltage;
  private final StatusSignal<Current> rollerRightSupply;
  private final StatusSignal<Current> rollerRightStator;
  private final StatusSignal<Temperature> rollerLeftTemperature;
  private final StatusSignal<Temperature> rollerRightTemperature;

  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotSupply;
  private final StatusSignal<Current> pivotStator;
  private final StatusSignal<Temperature> pivotTemperature;

  // divide the max free speed by the gear ratio to get the max pviot velocity
  private final AngularVelocity maxPivotVelocity =
      RadiansPerSecond.of(DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec).div(50.0);

  public IntakeIOTalonFX() {
    pivot = new TalonFX(RobotMap.IntakeMap.pivot, RobotMap.systemBus);
    rollerLeft = new TalonFX(RobotMap.IntakeMap.rollerLeft, RobotMap.systemBus);
    rollerRight = new TalonFX(RobotMap.IntakeMap.rollerRight, RobotMap.systemBus);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CurrentLimits.maxSupply.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    PhoenixUtil.tryUntilOk(5, () -> (rollerLeft.getConfigurator().apply(config, 5)));
    PhoenixUtil.tryUntilOk(5, () -> (rollerRight.getConfigurator().apply(config, 5)));

    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.CurrentLimits.maxStator.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // TODO: PID STUFF
    config.Slot0.kP = IntakeConstants.PID.kP.get();
    config.Slot0.kI = IntakeConstants.PID.kI.get();
    config.Slot0.kD = IntakeConstants.PID.kD.get();
    config.Slot0.kG = IntakeConstants.PID.kG.get();
    config.Slot0.kS = IntakeConstants.PID.kS.get();
    config.Slot0.kV = IntakeConstants.PID.kV.get();
    config.Slot0.kA = IntakeConstants.PID.kA.get();
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.Mechanical.gearing;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Voltage.PeakForwardVoltage = IntakeConstants.VoltageLimits.peakForwardVoltage.in(Volts);
    config.Voltage.PeakReverseVoltage = IntakeConstants.VoltageLimits.peakReverseVoltage.in(Volts);

    config.MotionMagic.MotionMagicAcceleration =
        maxPivotVelocity.per(Second).in(RotationsPerSecondPerSecond);
    config.MotionMagic.MotionMagicCruiseVelocity = maxPivotVelocity.in(RotationsPerSecond);

    PhoenixUtil.tryUntilOk(5, () -> (pivot.getConfigurator().apply(config, 5)));
    rollerVelocity = rollerLeft.getVelocity();
    rollerLeftVoltage = rollerLeft.getMotorVoltage();
    rollerLeftSupply = rollerLeft.getSupplyCurrent();
    rollerLeftStator = rollerLeft.getStatorCurrent();
    rollerRightVoltage = rollerRight.getMotorVoltage();
    rollerRightSupply = rollerRight.getSupplyCurrent();
    rollerRightStator = rollerRight.getStatorCurrent();
    rollerLeftTemperature = rollerLeft.getDeviceTemp();
    rollerRightTemperature = rollerRight.getDeviceTemp();

    pivotAngle = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotSupply = pivot.getSupplyCurrent();
    pivotStator = pivot.getStatorCurrent();
    pivotTemperature = pivot.getDeviceTemp();

    pivot.setPosition(IntakeConstants.Setpoints.stowed);

    StatusSignal.setUpdateFrequencyForAll(250.0, pivotAngle);
    StatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerLeftVoltage,
        rollerLeftSupply,
        rollerLeftStator,
        rollerRightVoltage,
        rollerRightSupply,
        rollerRightStator,
        rollerLeftTemperature,
        rollerRightTemperature,
        rollerVelocity,
        pivotVelocity,
        pivotVoltage,
        pivotSupply,
        pivotStator,
        pivotTemperature);
    PhoenixUtil.tryUntilOk(5, () -> rollerLeft.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> rollerRight.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> pivot.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    StatusSignal.refreshAll(
        rollerVelocity,
        rollerLeftVoltage,
        rollerLeftSupply,
        rollerLeftStator,
        rollerLeftTemperature,
        pivotAngle,
        pivotVelocity,
        pivotVoltage,
        pivotSupply,
        pivotStator,
        pivotTemperature);


    inputs.rollerLeftConnected =
        StatusSignal.isAllGood(
            rollerVelocity, rollerLeftVoltage, rollerLeftSupply, rollerLeftStator, rollerLeftTemperature);
    inputs.rollerRightConnected =
        StatusSignal.isAllGood(
          rollerRightVoltage, rollerRightSupply, rollerRightStator, rollerRightTemperature);
    inputs.rollerVelocity = rollerVelocity.getValue();
    inputs.rollerLeftAppliedVoltage = rollerLeftVoltage.getValue();
    inputs.rollerLeftStator = rollerLeftStator.getValue();
    inputs.rollerLeftSupply = rollerLeftSupply.getValue();
    inputs.rollerLeftTemp = rollerLeftTemperature.getValue();
    inputs.rollerRightAppliedVoltage = rollerRightVoltage.getValue();
    inputs.rollerRightStator = rollerRightStator.getValue();
    inputs.rollerRightSupply = rollerRightSupply.getValue();
    inputs.rollerRightTemp = rollerRightTemperature.getValue();

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

  @SuppressWarnings("unused")
  private void resetValues() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKP(kP.getAsDouble());
    slot0Configs.withKI(kI.getAsDouble());
    slot0Configs.withKD(kD.getAsDouble());
    slot0Configs.withKV(kV.getAsDouble());
    slot0Configs.withKA(kA.getAsDouble());
    slot0Configs.withKG(kG.getAsDouble());
    slot0Configs.withKS(kS.getAsDouble());
    pivot.getConfigurator().apply(slot0Configs, 0.25);
  }

  @Override
  public void setPosition(Angle angle) {
    pivot.setControl(motionMagic.withPosition(angle));
  }

  @Override
  public void setCurrent(Current desired) {
    rollerLeft.setControl(applyCurrent.withOutput(desired));
  }

  @Override
  public void setVoltage(Voltage applied) {
    rollerLeft.setControl(applyVoltage.withOutput(applied));
    rollerRight.setControl(applyVoltage.withOutput(applied.unaryMinus()));

    if (applied.magnitude() == 0) {
      rollerLeft.stopMotor();
      rollerRight.stopMotor();
    }
  }

  @Override
  public void setVoltageTest(Voltage applied, boolean left) {
    if (left) { 
      rollerLeft.setControl(applyVoltage.withOutput(applied));
    } else {
      rollerRight.setControl(applyVoltage.withOutput(applied.unaryMinus()));
    }
    if (applied.magnitude() == 0) {
      rollerLeft.stopMotor();
      rollerRight.stopMotor();
    }
  }

  @Override
  public void setPivotVoltage(Voltage applied) {
    Logger.recordOutput("Intake/Applied Volts", applied.in(Volts));
    pivot.setControl(applyPivotVoltage.withOutput(MathUtil.clamp(applied.in(Volts), -1, 1)));
    if (applied.magnitude() == 0) {
      pivot.stopMotor();
    }
  }
}

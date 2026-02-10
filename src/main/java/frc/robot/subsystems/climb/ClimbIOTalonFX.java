// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.KrakenX60;
import frc.robot.RobotMap;
import frc.robot.subsystems.climb.ClimbConstants.Position;
import org.littletonrobotics.junction.Logger;

public class ClimbIOTalonFX implements ClimbIO {

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private TalonFX climb;
  private TalonFXConfiguration config;
  private StatusSignal<Temperature> temp;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Voltage> voltageSignal;
  private double setpoint = 0.0;

  public ClimbIOTalonFX(int id) {
    climb = new TalonFX(id, RobotMap.systemBus);
    config = new TalonFXConfiguration();

    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimit = 80.0;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.PeakForwardDutyCycle = 0.8;
    config.MotorOutput.PeakReverseDutyCycle = 0.8;

    config.MotionMagic.withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second));
    config.MotionMagic.withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed);

    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond);
    config.Slot0.kS = 0.0;

    climb.getConfigurator().apply(config);

    temp = climb.getDeviceTemp();
    velocity = climb.getVelocity();
    positionSignal = climb.getPosition();
    statorCurrent = climb.getStatorCurrent();
    supplyCurrent = climb.getSupplyCurrent();
    voltageSignal = climb.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, temp, statorCurrent, supplyCurrent, voltageSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotMap.systemBus.isNetworkFD() ? 250.0 : 50.0, velocity, positionSignal);
    climb.optimizeBusUtilization();
    Logger.recordOutput("Robot Map/Climb ID", climb.getDeviceID());
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        temp, velocity, positionSignal, statorCurrent, supplyCurrent, voltageSignal);
    inputs.isConnected =
        BaseStatusSignal.isAllGood(
            temp, velocity, positionSignal, statorCurrent, supplyCurrent, voltageSignal);

    inputs.position = positionSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.voltage = voltageSignal.getValueAsDouble();
    inputs.setpoint = this.setpoint;
  }

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setPosition(Position position) {
    this.setpoint = this.motorAngleToExtension(position.motorAngle()).in(Inches);
    climb.setControl(motionMagicRequest.withPosition(position.motorAngle()));
  }

  private Distance motorAngleToExtension(Angle motorAngle) {
    final Measure<DistanceUnit> extensionMeasure =
        motorAngle.timesRatio(ClimbConstants.kHangerExtensionPerMotorAngle);
    return Inches.of(extensionMeasure.in(Inches));
  }
}

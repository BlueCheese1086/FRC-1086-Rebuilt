// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX upperLeft;
  private final TalonFX lowerLeft;
  private final TalonFX upperRight;
  private final TalonFX lowerRight;

  // Over all Mechanism stuff
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;

  private final StatusSignal<Voltage> uLVoltage;
  private final StatusSignal<Temperature> uLTemperature;
  private final StatusSignal<Current> uLSupply;
  private final StatusSignal<Current> uLStator;

  private final StatusSignal<Voltage> lLVoltage;
  private final StatusSignal<Temperature> lLTemperature;
  private final StatusSignal<Current> lLSupply;
  private final StatusSignal<Current> lLStator;

  private final StatusSignal<Voltage> uRVoltage;
  private final StatusSignal<Temperature> uRTemperature;
  private final StatusSignal<Current> uRSupply;
  private final StatusSignal<Current> uRStator;

  private final StatusSignal<Voltage> lRVoltage;
  private final StatusSignal<Temperature> lRTemperature;
  private final StatusSignal<Current> lRSupply;
  private final StatusSignal<Current> lRStator;

  private final StatusSignal<Integer> uLVersion;
  private final StatusSignal<Integer> lLVersion;
  private final StatusSignal<Integer> uRVersion;
  private final StatusSignal<Integer> lRVersion;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);
  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);

  TalonFXConfiguration configuration = new TalonFXConfiguration();

  public boolean useMotionMagic = false;

  public ShooterIOTalonFX() {
    upperLeft = new TalonFX(RobotMap.Shooter.UpperLeft, RobotMap.systemBus);
    lowerLeft = new TalonFX(RobotMap.Shooter.LowerLeft, RobotMap.systemBus);
    upperRight = new TalonFX(RobotMap.Shooter.UpperRight, RobotMap.systemBus);
    lowerRight = new TalonFX(RobotMap.Shooter.LowerRight, RobotMap.systemBus);
    configuration.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.CurrentLimits.maxStator.in(Amps);
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.CurrentLimits.maxStator.in(Amps);
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    configuration.Voltage.PeakForwardVoltage = ShooterConstants.VoltageLimits.maxVoltage.in(Volts);
    configuration.Voltage.PeakReverseVoltage = ShooterConstants.VoltageLimits.minVoltage.in(Volts);
    configuration.Feedback.SensorToMechanismRatio = ShooterConstants.Mechanical.gearing;
    configuration.MotorOutput.PeakForwardDutyCycle = 1.0;
    configuration.MotorOutput.PeakReverseDutyCycle = -1.0;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configuration.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        0.5; // Hopefully prevents drum motors from killing themselves
    configuration.Slot0.kP = 0.1;
    configuration.Slot0.kI = 0.0;
    configuration.Slot0.kD = 0.0;
    configuration.Slot0.kS = 0.42122;
    configuration.Slot0.kG = 0.0;
    configuration.Slot0.kV = 0.11915;
    configuration.Slot0.kA = 0.020051;
    configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        0.5; // Hopefully prevents drum motors from killing themselves

    PhoenixUtil.tryUntilOk(5, () -> upperLeft.getConfigurator().apply(configuration, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> lowerLeft.getConfigurator().apply(configuration, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> upperRight.getConfigurator().apply(configuration, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> lowerRight.getConfigurator().apply(configuration, 0.25));

    position = upperLeft.getPosition();
    velocity = upperLeft.getVelocity();
    acceleration = upperLeft.getAcceleration();

    uLVersion = upperLeft.getVersion();
    uLVoltage = upperLeft.getMotorVoltage();
    uLTemperature = upperLeft.getDeviceTemp();
    uLStator = upperLeft.getStatorCurrent();
    uLSupply = upperLeft.getSupplyCurrent();

    lLVersion = lowerLeft.getVersion();
    lLVoltage = lowerLeft.getMotorVoltage();
    lLTemperature = lowerLeft.getDeviceTemp();
    lLStator = lowerLeft.getStatorCurrent();
    lLSupply = lowerLeft.getSupplyCurrent();

    uRVersion = upperRight.getVersion();
    uRVoltage = upperRight.getMotorVoltage();
    uRTemperature = upperRight.getDeviceTemp();
    uRStator = upperRight.getStatorCurrent();
    uRSupply = upperRight.getSupplyCurrent();

    lRVersion = lowerRight.getVersion();
    lRVoltage = lowerRight.getMotorVoltage();
    lRTemperature = lowerRight.getDeviceTemp();
    lRStator = lowerRight.getStatorCurrent();
    lRSupply = lowerRight.getSupplyCurrent();

    PhoenixUtil.tryUntilOk(
        5, () -> StatusSignal.setUpdateFrequencyForAll(50, position, velocity, acceleration));

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            StatusSignal.setUpdateFrequencyForAll(
                50,
                uLVoltage,
                uLTemperature,
                uLStator,
                uLSupply,
                lLVoltage,
                lLTemperature,
                lLStator,
                lLSupply,
                uRVoltage,
                uRTemperature,
                uRStator,
                uRSupply,
                lRVoltage,
                lRTemperature,
                lRStator,
                lRSupply));

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            StatusSignal.setUpdateFrequencyForAll(
                50,
                upperLeft.getControlMode(),
                lowerLeft.getControlMode(),
                upperRight.getControlMode(),
                lowerRight.getControlMode()));

    PhoenixUtil.tryUntilOk(
        5,
        () -> StatusSignal.setUpdateFrequencyForAll(4, uLVersion, lLVersion, uRVersion, lRVersion));

    PhoenixUtil.tryUntilOk(5, () -> upperLeft.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> lowerLeft.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> upperRight.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> lowerRight.optimizeBusUtilization());

    upperRight.setControl(new Follower(upperLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    lowerLeft.setControl(new Follower(upperLeft.getDeviceID(), MotorAlignmentValue.Aligned));
    lowerRight.setControl(new Follower(upperRight.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    StatusSignal.refreshAll(
        position,
        velocity,
        acceleration,
        uLVoltage,
        uLTemperature,
        uLStator,
        uLSupply,
        lLVoltage,
        lLTemperature,
        lLStator,
        lLSupply,
        uRVoltage,
        uRTemperature,
        uRStator,
        uRSupply,
        lRVoltage,
        lRTemperature,
        lRStator,
        lRSupply);

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue().in(RadiansPerSecond);
    inputs.acceleration = acceleration.getValueAsDouble();

    // Upper Left
    inputs.upperLeftConnected = upperLeft.isConnected();
    inputs.upperLeftAlive = StatusSignal.isAllGood(uLVoltage, uLTemperature, uLStator, uLSupply);
    inputs.upperLeftVoltage = uLVoltage.getValue();
    inputs.upperLeftTemperature = uLTemperature.getValue();
    inputs.upperLeftStator = uLStator.getValue();
    inputs.upperLeftSupply = uLSupply.getValue();

    // Lower Left
    inputs.lowerLeftConnected = lowerLeft.isConnected();
    inputs.lowerLeftAlive = StatusSignal.isAllGood(lLVoltage, lLTemperature, lLStator, lLSupply);
    inputs.lowerLeftVoltage = lLVoltage.getValue();
    inputs.lowerLeftTemperature = lLTemperature.getValue();
    inputs.lowerLeftStator = lLStator.getValue();
    inputs.lowerLeftSupply = lLSupply.getValue();

    // Upper Right
    inputs.upperRightConnected = upperRight.isConnected();
    inputs.upperRightAlive = StatusSignal.isAllGood(uRVoltage, uRTemperature, uRStator, uRSupply);
    inputs.upperRightVoltage = uRVoltage.getValue();
    inputs.upperRightTemperature = uRTemperature.getValue();
    inputs.upperRightStator = uRStator.getValue();
    inputs.upperRightSupply = uRSupply.getValue();

    // Lower Right
    inputs.lowerRightConnected = lowerRight.isConnected();
    inputs.lowerRightAlive = StatusSignal.isAllGood(lRVoltage, lRTemperature, lRStator, lRSupply);
    inputs.lowerRightVoltage = lRVoltage.getValue();
    inputs.lowerRightTemperature = lRTemperature.getValue();
    inputs.lowerRightStator = lRStator.getValue();
    inputs.lowerRightSupply = lRSupply.getValue();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    upperLeft.setControl(
        useMotionMagic
            ? motionMagic.withVelocity(velocity)
            : velocityControl.withVelocity(velocity));
  }

  @Override
  public void setVoltage(double volts) {
    upperLeft.setControl(voltageControl.withOutput(volts));
  }
}

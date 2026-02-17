// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;

/** Add your docs here. */
public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX climbTalon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private StatusSignal<Angle> angle;
  private StatusSignal<AngularVelocity> angularVelocity;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Temperature> temp;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0); // fix
  private VoltageOut voltagething = new VoltageOut(0);

  private final Debouncer connected = new Debouncer(1.0); // What does this do?

  private double targetPosition;

  public ClimbIOTalonFX() {
    this.climbTalon = new TalonFX(RobotMap.climber, RobotMap.systemBus);

    config.MotorOutput.Inverted = ClimbConstants.invertedValue;
    config.MotorOutput.NeutralMode = ClimbConstants.neutralMode;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;
    config.Feedback.SensorToMechanismRatio =
        ClimbConstants.gearing
            / ClimbConstants
                .radius; // This will all make sense because Kraken does 1/Ratio for you.

    CurrentLimitsConfigs limitConfig = config.CurrentLimits;

    limitConfig.withStatorCurrentLimit(Amps.of(60)); // Replace with a value pls
    limitConfig.withStatorCurrentLimitEnable(true);
    limitConfig.withSupplyCurrentLimit(Amps.of(60)); // also this onee
    limitConfig.withSupplyCurrentLimitEnable(true);

    Slot0Configs slotConfig = config.Slot0;

    slotConfig.kG = ClimbConstants.kG; // gravity gains
    slotConfig.kS = ClimbConstants.kS; // static friction gains

    slotConfig.kV = ClimbConstants.kV; // output velocity
    slotConfig.kA = ClimbConstants.kA; // acceleration
    slotConfig.kP = ClimbConstants.kP;
    slotConfig.kI = ClimbConstants.kI;
    slotConfig.kD = ClimbConstants.kD;

    MotionMagicConfigs magic = config.MotionMagic;

    magic.withMotionMagicAcceleration(ClimbConstants.krakenFreeSpeed.per(Second));
    magic.withMotionMagicCruiseVelocity(ClimbConstants.krakenFreeSpeed);
    magic.MotionMagicJerk = 0.0; // fix

    climbTalon.getConfigurator().apply(config);

    this.angle = climbTalon.getPosition();
    this.angularVelocity = climbTalon.getVelocity();
    this.volts = climbTalon.getMotorVoltage();
    this.temp = climbTalon.getDeviceTemp();
    this.statorCurrent = climbTalon.getStatorCurrent();
    this.supplyCurrent = climbTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, angle, angularVelocity, volts, temp, statorCurrent, supplyCurrent);
    climbTalon.optimizeBusUtilization();

    // Stator, supply,vel, accl, temp
    // Stator: torque?
    // supply: how much $ getting, how much giving to torque?
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    StatusCode status =
        BaseStatusSignal.refreshAll(
            angle, angularVelocity, volts, temp, statorCurrent, supplyCurrent);

    inputs.motorConnected = connected.calculate(status.isOK());

    inputs.targetPosition = this.targetPosition;
    inputs.angle = angle.getValue();
    inputs.velocity = angularVelocity.getValue();
    inputs.volts = volts.getValue();
    inputs.temp = temp.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.climbPosition = angle.getValueAsDouble();
  }

  @Override
  public void resetEncoder() {
    climbTalon.setPosition(0.0); // is this double or angle? either works because it is 0 - Martin
  }

  @Override
  public void setVoltage(double volts) {
    climbTalon.setControl(voltagething.withOutput(volts));
    // I am pretty sure this is how you make it move
    // I just don't know what to put in off the top of my head without copying it from somewhere
  }

  @Override
  public void setPosition(double position) {
    this.targetPosition = position;
    climbTalon.setControl(motionMagic.withPosition(position)); // is this right?
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
  private final TalonFXConfiguration config;
  private StatusSignal<Angle> angle;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Temperature> temp;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0); // fix
  private VoltageOut voltagething;

  private final Debouncer connected = new Debouncer(1.0); // What does this do?

  private double targetPosition;

  public ClimbIOTalonFX() {
    this.config = new TalonFXConfiguration();
    this.climbTalon = new TalonFX(RobotMap.climber, RobotMap.systemBus);
    this.voltagething = new VoltageOut(0); // relace with value pls

    config.MotorOutput.Inverted = ClimbConstants.invertedValue;
    config.MotorOutput.NeutralMode = ClimbConstants.neutralMode;
    config.MotorOutput.PeakForwardDutyCycle = 0.8;
    config.MotorOutput.PeakReverseDutyCycle = 0.8;
    config.Feedback.SensorToMechanismRatio =
        ClimbConstants.gearing / (2 * Math.PI * ClimbConstants.radius);

    var limitConfig = config.CurrentLimits;

    limitConfig.withStatorCurrentLimit(Amps.of(80)); // Replace with a value pls
    limitConfig.withStatorCurrentLimitEnable(true);
    limitConfig.withSupplyCurrentLimit(Amps.of(80)); // also this onee
    limitConfig.withSupplyCurrentLimitEnable(true);

    var slotConfig = config.Slot0;

    slotConfig.kG = ClimbConstants.kG; // gravity gains
    slotConfig.kS = ClimbConstants.kS; // static friction gains

    slotConfig.kV = ClimbConstants.kV; // output velocity
    slotConfig.kA = ClimbConstants.kA; // acceleration
    slotConfig.kP = ClimbConstants.kP;
    slotConfig.kI = ClimbConstants.kI;
    slotConfig.kD = ClimbConstants.kD;

    var magic = config.MotionMagic;

    magic.withMotionMagicAcceleration(ClimbConstants.krackenFreeSpeed.per(Second));
    magic.withMotionMagicCruiseVelocity(ClimbConstants.krackenFreeSpeed);
    magic.MotionMagicJerk = 0.0; // fix

    climbTalon.getConfigurator().apply(config);

    this.angle = climbTalon.getPosition();
    this.velocity = climbTalon.getVelocity();
    this.volts = climbTalon.getMotorVoltage();
    this.temp = climbTalon.getDeviceTemp();
    this.statorCurrent = climbTalon.getStatorCurrent();
    this.supplyCurrent = climbTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, angle, velocity, volts, temp, statorCurrent, supplyCurrent);

    climbTalon.optimizeBusUtilization();
    // Stator, supply,vel, accl, temp
    // Stator: torque?
    // supply: how much $ getting, how much giving to torque?

  }

  @Override
  public void setPosition(double position) {
    this.targetPosition = position;
    climbTalon.setControl(motionMagic.withPosition(position)); // is this right?
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(angle, velocity, volts, temp, statorCurrent, supplyCurrent);

    inputs.motorConnected = connected.calculate(status.isOK());

    inputs.targetPosition = this.targetPosition;
    inputs.angle = angle.getValue();
    inputs.volts = volts.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.climbPosition = angle.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
  }

  @Override
  public void resetEncoder() {
    climbTalon.setPosition(0.0); // is this double or angle?
  }

  @Override
  public void setVoltage(double volts) {
    climbTalon.setControl(voltagething.withOutput(volts));
    // I am pretty sure this is how you make it move
    // I just don't know what to put in off the top of my head without copying it from somewhere
  }
}

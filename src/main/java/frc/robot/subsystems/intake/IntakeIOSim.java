// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

/** Simulation IO implementation for the intake pivot and rollers. */
public class IntakeIOSim implements IntakeIO {
  private final PIDController pid = new PIDController(1.0, IntakeConstants.PID.kI.get(), 0.0);
  private final ArmFeedforward ff =
      new ArmFeedforward(IntakeConstants.PID.kS.get(), 0.0, IntakeConstants.PID.kV.get());

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          IntakeConstants.Mechanical.gearing,
          0.04,
          IntakeConstants.Mechanical.intakeLength.in(Meters),
          IntakeConstants.Setpoints.deployed.in(Radians),
          IntakeConstants.Setpoints.stowed.in(Radians),
          false,
          IntakeConstants.Setpoints.stowed.in(Radians));
  private final DCMotorSim simRoller =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.04, 1.0),
          DCMotor.getKrakenX60Foc(1));
  private boolean useClosedLoop = false;
  private double armInputVolts = 0.0;

  /** Creates the simulated intake IO and initializes the pivot setpoint. */
  public IntakeIOSim() {
    pid.setSetpoint(armInputVolts);
  }

  /**
   * Advances the intake simulation and stores simulated values in the input snapshot.
   *
   * @param inputs mutable input snapshot to fill
   */
  @Override
  public void updateInputs(IntakeInputs inputs) {
    if (useClosedLoop) {
      armSim.setInputVoltage(
          pid.calculate(armSim.getAngleRads()) + ff.calculate(pid.getSetpoint(), 0.0));
    } else {
      Logger.recordOutput("Intake/Pivot Volts", armInputVolts);
      armSim.setInputVoltage(armInputVolts);
    }

    armSim.update(0.02);
    simRoller.update(0.02);

    inputs.pivotAngle = Radians.of(armSim.getAngleRads());
    inputs.pivotVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
    inputs.pivotAppliedVoltage = Volts.of(armInputVolts);

    inputs.rollerLeftAppliedVoltage = Volts.of(simRoller.getInputVoltage());
    inputs.rollerLeftVelocity = simRoller.getAngularVelocityRadPerSec();
  }

  /**
   * Enables closed-loop pivot simulation to the requested angle.
   *
   * @param angle desired pivot angle
   */
  @Override
  public void setPosition(Angle angle) {
    useClosedLoop = true;
    pid.setSetpoint(angle.in(Radians));
  }

  /**
   * Applies voltage to the simulated roller.
   *
   * @param applied desired roller voltage
   */
  @Override
  public void setVoltage(Voltage applied) {
    simRoller.setInputVoltage(applied.in(Volts));
  }

  /**
   * Applies open-loop voltage to the simulated pivot.
   *
   * @param applied desired pivot voltage
   */
  @Override
  public void setPivotVoltage(Voltage applied) {
    useClosedLoop = false;
    Logger.recordOutput("Intake/Applied Volts", applied.in(Volts));
    armInputVolts = applied.in(Volts);
    armSim.setInputVoltage(armInputVolts);
  }
}

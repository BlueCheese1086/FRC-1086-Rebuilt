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

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
  private final PIDController pid =
      new PIDController(IntakeConstants.PID.kP, IntakeConstants.PID.kI, IntakeConstants.PID.kD);
  private final ArmFeedforward ff =
      new ArmFeedforward(IntakeConstants.PID.kS, 0.0, IntakeConstants.PID.kV);

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
  private boolean useClosedLoop = true;
  private double armInputVolts = 0.0;

  public IntakeIOSim() {
    pid.setSetpoint(armInputVolts);
  }

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
    inputs.pivotAppliedVoltage = Volts.of(armSim.getInput(0));

    inputs.rollerAppliedVoltage = Volts.of(simRoller.getInputVoltage());
    inputs.rollerVelocity = RadiansPerSecond.of(simRoller.getAngularVelocityRadPerSec());
  }

  @Override
  public void setPosition(Angle angle) {
    useClosedLoop = true;
    pid.setSetpoint(angle.in(Radians));
  }

  @Override
  public void setVoltage(Voltage applied) {
    simRoller.setInputVoltage(applied.in(Volts));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;
  private final IntakeInputs inputs = new IntakeInputs();

  private double setpoint = 0.0;
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command setPosition(Angle newPos) {
    return this.run(() -> {
      setpoint = newPos.in(Radians);
      io.setPosition(newPos);
    }).until(this::atSetpoint);
  }

  public Command setVoltage(Voltage applied) {
    return this.run(() -> {
      io.setVoltage(applied);
    }).finallyDo(() -> {
      io.setVoltage(Volts.zero());
    });
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(setpoint, inputs.pivotAngle.in(Radians), 0.1); // TODO: Tune this to require it to be more accurate.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }
}

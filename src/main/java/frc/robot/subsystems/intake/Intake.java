// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private Angle setpoint = Radians.zero();
  private SysIdRoutine routine;
  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
    routine =
        new SysIdRoutine(
            new Config(Volts.of(1).per(Second), Volts.of(3), Seconds.of(4)),
            new Mechanism(
                io::setPivotVoltage,
                (log) -> {
                  log.motor("Intake Pivot")
                      .angularPosition(inputs.pivotPosition)
                      .angularVelocity(inputs.pivotVelocity)
                      .voltage(inputs.pivotVoltage);
                },
                this));
  }

  public Command setPositionAsync(Angle angle) {
    return this.startRun(
            () -> {
              this.setpoint = angle;
            },
            () -> {
              io.setPivotPosition(angle);
            })
        .until(() -> true);
  }

  public Command setPosition(Angle angle) {
    return this.startRun(
            () -> {
              this.setpoint = angle;
            },
            () -> {
              io.setPivotPosition(angle);
            })
        .until(this::atSetpoint);
  }

  public Command runVoltage(Voltage voltage) {
    // Pivot is prioritized over this.
    return Commands.run(
            () -> {
              io.setRollerVoltage(voltage);
            })
        .finallyDo(
            () -> {
              io.setRollerVoltage(Volts.zero());
            });
  }

  public Command sysid() {
    return Commands.sequence(
        routine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    (MathUtil.isNear(
                        IntakeConstants.Pivot.retracted.in(Degrees),
                        inputs.pivotPosition.in(Degrees),
                        3.5))),
        routine
            .quasistatic(Direction.kReverse)
            .until(() -> (MathUtil.isNear(0, inputs.pivotPosition.in(Degrees), 3.5))),
        routine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    (MathUtil.isNear(
                        IntakeConstants.Pivot.retracted.in(Degrees),
                        inputs.pivotPosition.in(Degrees),
                        3.5))),
        routine
            .dynamic(Direction.kReverse)
            .until(() -> (MathUtil.isNear(0, inputs.pivotPosition.in(Degrees), 3.5))));
  }

  public Angle getPivotAngle() {
    return inputs.pivotPosition;
  }

  private boolean atSetpoint() {
    return MathUtil.isNear(setpoint.in(Degrees), inputs.pivotPosition.in(Degrees), 5);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Setpoint", setpoint);
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    inputs.pivotPositionDeg = inputs.pivotPosition.in(Degrees);
    Logger.processInputs("Intake", inputs);
  }
}

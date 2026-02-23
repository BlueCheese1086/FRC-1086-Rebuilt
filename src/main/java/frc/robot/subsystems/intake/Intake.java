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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;

  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private final SysIdRoutine routine;

  private double setpoint = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Second), Volts.of(1), Seconds.of(5.0)),
            new Mechanism(
                (applied) -> {
                  io.setPivotVoltage(Volts.of(MathUtil.clamp(applied.in(Volts), -1.0, 1.0)));
                },
                (log) -> {
                  log.motor("Pivot")
                      .angularPosition(inputs.pivotAngle)
                      .angularVelocity(inputs.pivotVelocity)
                      .voltage(inputs.pivotAppliedVoltage);
                },
                this));
  }

  public Command setPosition(Angle newPos) {
    return Commands.run(
            () -> {
              setpoint = newPos.in(Radians);
              io.setPosition(newPos);
            })
        .until(this::atSetpoint);
  }

  // public Command switchMode() {}

  public Command setVoltage(Voltage applied) {
    return Commands.run(
            () -> {
              io.setVoltage(applied);
            })
        .finallyDo(
            () -> {
              io.setVoltage(Volts.zero());
            });
  }

  public Command setPivotVoltage(Voltage applied) {
    return Commands.run(
            () -> {
              io.setPivotVoltage(applied);
            })
        .finallyDo(
            () -> {
              io.setPivotVoltage(Volts.zero());
            });
  }

  public Command setCurrent(Current applied) {
    return Commands.run(
            () -> {
              io.setCurrent(applied);
            })
        .finallyDo(
            () -> {
              io.setVoltage(Volts.zero());
            });
  }

  public Command sysId() {
    return Commands.sequence(
        routine
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    (MathUtil.isNear(
                        -4,
                        inputs.pivotAngle.in(Degrees),
                        IntakeConstants.Mechanical.kPositionTolerance.in(
                            Degrees)))), // TODO: Double Check This
        routine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    (MathUtil.isNear(
                        110,
                        inputs.pivotAngle.in(Degrees),
                        IntakeConstants.Mechanical.kPositionTolerance.in(Degrees)))),
        routine
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    (MathUtil.isNear(
                        -4,
                        inputs.pivotAngle.in(Degrees),
                        IntakeConstants.Mechanical.kPositionTolerance.in(Degrees)))),
        routine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    (MathUtil.isNear(
                        110,
                        inputs.pivotAngle.in(Degrees),
                        IntakeConstants.Mechanical.kPositionTolerance.in(Degrees)))));
  }

  @AutoLogOutput(key = "Intake/Near Setpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(
        setpoint,
        inputs.pivotAngle.in(Radians),
        IntakeConstants.Mechanical.kPositionTolerance.in(
            Radians)); // TODO: Tune this to require it to be more accurate.
  }

  public Command switchMode() {
    return Commands.runOnce(
        () -> {
          io.switchMode();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    inputs.nearSetpoint = atSetpoint();
    Logger.processInputs("Intake", inputs);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Command-based subsystem for controlling the intake pivot and rollers. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;

  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private final SysIdRoutine routine;
  private final BatteryLogger logger;

  private double setpoint = 0.0;

  /**
   * Creates an intake subsystem.
   *
   * @param io hardware or simulation IO layer
   * @param logger battery/current logger
   */
  public Intake(IntakeIO io, BatteryLogger logger) {
    this.io = io;
    this.logger = logger;
    routine =
        new SysIdRoutine(
            new Config(Volts.of(1).per(Second), Volts.of(4), Seconds.of(5.0)),
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

  /**
   * Commands the pivot to a target angle.
   *
   * @param newPos desired pivot angle
   * @return command that finishes when the pivot reaches the setpoint
   */
  public Command setPosition(Angle newPos) {
    return Commands.runOnce(
            () -> {
              setpoint = newPos.in(Radians);
              io.setPosition(newPos);
            })
        .until(this::atSetpoint);
  }

  /**
   * Runs the intake rollers at an applied voltage.
   *
   * @param applied roller voltage
   * @return command that stops the rollers when interrupted or ended
   */
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

  /**
   * Sets the roller closed-loop velocity once.
   *
   * @param velocity desired roller velocity
   * @return instant command that applies the velocity target
   */
  public Command runVelocity(AngularVelocity velocity) {
    return this.runOnce(() -> io.setRollerVelocity(velocity));
  }

  /**
   * Stops the intake roller velocity command.
   *
   * @return instant command that sets roller velocity to zero
   */
  public Command stopRoller() {
    return this.runOnce(() -> io.setRollerVelocity(RadiansPerSecond.of(0.0)));
  }

  /**
   * Runs the pivot at an applied voltage.
   *
   * @param applied pivot voltage
   * @return command that zeros pivot voltage when interrupted or ended
   */
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

  /**
   * Runs the rollers with torque-current control.
   *
   * @param applied desired roller current
   * @return command that stops roller voltage when interrupted or ended
   */
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

  /**
   * Runs the intake pivot characterization routine.
   *
   * @return command sequence for dynamic and quasistatic SysId tests
   */
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

  /**
   * Checks whether the pivot is within the configured position tolerance.
   *
   * @return true when the current pivot angle is near the active setpoint
   */
  @AutoLogOutput(key = "Intake/Near Setpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(
        setpoint,
        inputs.pivotAngle.in(Radians),
        IntakeConstants.Mechanical.kPositionTolerance.in(
            Radians)); // TODO: Tune this to require it to be more accurate.
  }

  /**
   * Toggles the IO control mode.
   *
   * @return instant command that requests the IO mode switch
   */
  public Command switchMode() {
    return Commands.runOnce(
        () -> {
          io.switchMode();
        });
  }

  /** Updates intake sensor inputs, logs telemetry, and reports current usage. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    inputs.nearSetpoint = atSetpoint();
    Logger.processInputs("Intake", inputs);

    logger.reportCurrentUsage("Intake/Pivot", false, inputs.pivotSupply.in(Amps));
    logger.reportCurrentUsage("Intake/Left Roller", false, inputs.rollerLeftSupply.in(Amps));
    logger.reportCurrentUsage("Intake/Right Roller", false, inputs.rollerRightSupply.in(Amps));
  }
}

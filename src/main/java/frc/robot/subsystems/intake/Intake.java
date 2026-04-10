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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.BatteryLogger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;

  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private final SysIdRoutine routine;
  private final BatteryLogger logger;

  private double setpoint = 0.0;

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

  public Command setPosition(Angle newPos) {
    return Commands.runOnce(
            () -> {
              setpoint = newPos.in(Radians);
              io.setPosition(newPos);
            })
        .until(this::atSetpoint);
  }

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

  public Command runVelocity(AngularVelocity velocity) {
    return this.runOnce(() -> io.setRollerVelocity(velocity));
  }

  public Command stopRoller() {
    return this.runOnce(() -> io.setRollerVelocity(RadiansPerSecond.of(0.0)));
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

  /**
   * Smoothly move the intake from the deployed setpoint up to the stowed setpoint over the
   * specified duration (seconds).
   *
   * @param seconds duration in seconds for the motion
   */
  public Command smush(double seconds) {
    final Timer timer = new Timer();
    final Angle start = IntakeConstants.Setpoints.deployed;
    final Angle end = IntakeConstants.Setpoints.stowed;

    return Commands.sequence(
        // start timer
        Commands.runOnce(
            () -> {
              timer.reset();
              timer.start();
            }),
        // on each scheduler run, compute interpolated angle and command it to the IO
        Commands.run(
                () -> {
                  double frac = MathUtil.clamp(timer.get() / seconds, 0.0, 1.0);
                  double s = start.in(Radians);
                  double e = end.in(Radians);
                  double interp = s + (e - s) * frac;
                  io.setPosition(Radians.of(interp));
                },
                this)
            .withTimeout(seconds),
        // ensure final position and stop timer
        Commands.runOnce(
            () -> {
              timer.stop();
              io.setPosition(end);
            }));
  }

  public Command agitate() {
    return Commands.repeatingSequence(
        Commands.runOnce(() -> io.setPosition(IntakeConstants.Setpoints.agitate), this),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> io.setPosition(IntakeConstants.Setpoints.deployed), this),
        Commands.waitSeconds(0.5));
  }

  // Cleaning mode
  public Command clean() {
    return setVoltage(Volts.of(1.5));
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

    logger.reportCurrentUsage("Intake/Pivot", false, inputs.pivotSupply.in(Amps));
    logger.reportCurrentUsage("Intake/Left Roller", false, inputs.rollerLeftSupply.in(Amps));
    logger.reportCurrentUsage("Intake/Right Roller", false, inputs.rollerRightSupply.in(Amps));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final ClimbIO io;
  private final SysIdRoutine routine;

  public Climb(ClimbIO io) {
    this.io = io;
    io.resetEncoder();
    this.inputs = new ClimbIOInputsAutoLogged();
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1.0).per(Second), Volts.of(4.0), Seconds.of(5.0)),
            new SysIdRoutine.Mechanism(
                (applied) -> {
                  io.setVoltage(applied.in(Volts));
                },
                (log) -> {
                  log.motor("Climber")
                      .linearPosition(Meters.of(inputs.angle.in(Radians)))
                      .linearVelocity(MetersPerSecond.of(inputs.velocity))
                      .voltage(Volts.of(inputs.volts));
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public Command setPosition(double position) {
    return this.run(
            () -> {
              inputs.targetPosition = position;
              io.setPosition(
                  MathUtil.clamp(
                      position, ClimbConstants.retractedHeight, ClimbConstants.extendedHeight));
            })
        .until(this::atSetpoint);
  }

  public Command setPositionDynamic(
      DoubleSupplier
          position) { // We probably won't need this but I am going to leave it in anyways.
    return this.run(
            () -> {
              inputs.targetPosition = position.getAsDouble();
              io.setPosition(
                  MathUtil.clamp(
                      position.getAsDouble(),
                      ClimbConstants.retractedHeight,
                      ClimbConstants.extendedHeight));
            })
        .until(this::atSetpoint);
  }

  @AutoLogOutput(key = "Climb/At Setpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.targetPosition, inputs.climbPosition, 0.01);
  }

  public Command resetEncoder() {
    return Commands.runOnce(() -> io.resetEncoder()).ignoringDisable(true);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine
        .dynamic(direction)
        .until(
            () -> {
              return direction.equals(Direction.kForward)
                  ? MathUtil.isNear(
                      ClimbConstants.extendedHeight,
                      inputs.climbPosition,
                      ClimbConstants.SysIdTolerance)
                  : MathUtil.isNear(
                      ClimbConstants.retractedHeight,
                      inputs.climbPosition,
                      ClimbConstants.SysIdTolerance);
            });
  }

  public Command sysId() {
    return Commands.sequence(
        routine.dynamic(Direction.kForward).withTimeout(2.0),
            routine.dynamic(Direction.kReverse).withTimeout(2.0),
        routine.quasistatic(Direction.kForward).withTimeout(2.0),
            routine.quasistatic(Direction.kReverse).withTimeout(2.0));
  }

  public Command sysIdQuasistic(SysIdRoutine.Direction direction) {
    return routine
        .quasistatic(direction)
        .until(
            () -> {
              return direction.equals(Direction.kForward)
                  ? MathUtil.isNear(
                      ClimbConstants.extendedHeight,
                      inputs.climbPosition,
                      ClimbConstants.SysIdTolerance)
                  : MathUtil.isNear(
                      ClimbConstants.retractedHeight,
                      inputs.climbPosition,
                      ClimbConstants.SysIdTolerance);
            });
  }
}

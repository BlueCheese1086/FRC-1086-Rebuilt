// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private final HoodIO io;

  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

  private Angle setpoint = Degrees.zero();

  private final SysIdRoutine routine;

  public Hood(HoodIO io) {
    this.io = io;
    routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Second), Volts.of(2), Second.of(4)),
            new Mechanism(
                io::setVoltage,
                (log) -> {
                  log.motor("Hood")
                      .angularPosition(inputs.position)
                      .angularVelocity(inputs.velocity)
                      .voltage(inputs.appliedVoltage);
                },
                this));
  }

  public Command setAngle(Supplier<Angle> angle) {
    return this.run(
        () -> {
          this.setpoint = angle.get();
          io.setAngle(
              Radians.of(
                  MathUtil.clamp(
                      angle.get().in(Radians),
                      0.0,
                      Units.degreesToRadians(HoodConstants.Mechanical.travel))));
        });
  }

  public Command runAngleThenLower(Supplier<Angle> angle) {
    return this.run(
            () -> {
              this.setpoint = angle.get();
              io.setAngle(
                  Radians.of(
                      MathUtil.clamp(
                          angle.get().in(Radians),
                          0.0,
                          Units.degreesToRadians(HoodConstants.Mechanical.travel))));
            })
        .finallyDo(
            () -> {
              this.setpoint = Radians.zero();
              io.setAngle(Radians.zero());
            });
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(setpoint.in(Degrees), inputs.position.in(Degrees), 5);
  }

  @AutoLogOutput(key = "Hood/Hood Angle")
  public Angle getAngle() {
    return inputs.position.plus(Degrees.of(7.402304));
  }

  public Command runVoltage(Voltage voltage) {
    return this.runEnd(
        () -> {
          io.setVoltage(voltage);
        },
        () -> {
          io.setVoltage(Volts.zero());
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    inputs.positionDeg = inputs.position.in(Degrees);
    Logger.processInputs("Hood", inputs);
  }

  public Command resetEncoder() {
    return this.runOnce(io::resetEncoder).ignoringDisable(true);
  }

  public Command autoZero() {
    return Commands.sequence(
        this.runVoltage(Volts.of(-1)).until(() -> inputs.velocity.in(RadiansPerSecond) < 0.1),
        resetEncoder());
  }
}

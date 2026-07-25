// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.FeederIO.FeederIO;
import frc.robot.subsystems.shooter.FeederIO.FeederIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterInputsAutoLogged shooterIOinputsAutoLogged;
  private ShooterIO shooterIO;
  private FeederIO feederIO;
  private FeederIOInputsAutoLogged feederIOInputsAutoLogged;
  // private final File file;

  public Shooter(FeederIO feederIO, ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    this.feederIO = feederIO;
    this.feederIOInputsAutoLogged = new FeederIOInputsAutoLogged();
    this.shooterIOinputsAutoLogged = new ShooterInputsAutoLogged();
  }

  /**
   * Sets the velocity setpoint of the shooter. This is the value that the shooter will try to get
   * to.
   *
   * @param radPerSec The new velocity setpoint
   */
  public void setVelocitySetpoint(Supplier<AngularVelocity> radPerSec) {
    shooterIO.setVelocity(radPerSec.get());
  }

  /**
   * Runs the feeder at a specified voltage. Intended to be used within safe tolerances.
   *
   * @param volts the desired output voltage
   */
  public Command runFeed(double volts) {
    return Commands.run(() -> feederIO.setFeedVoltage(volts)).finallyDo(() -> stopFeeder());
  }

  /**
   * Sets the voltage of the shooter motors
   *
   * @param volts the request output voltage
   */
  public Command setVoltage(double volts) {
    return this.run(
            () -> {
              shooterIO.setVoltage(volts);
            })
        .finallyDo(this::stopShooter);
  }

  /** Stops both the feeder and shooter motors */
  public void stopAll() {
    shooterIO.setVoltage(0.0);
    shooterIO.setVelocity(RadiansPerSecond.of(0.0));
    feederIO.setFeedVoltage(0.0);
  }

  /** Stops only the shooter */
  public void stopShooter() {
    shooterIO.setVelocity(RadiansPerSecond.of(0.0));
  }

  /** Stops only the feeder */
  public void stopFeeder() {
    feederIO.setFeedVoltage(0.0);
  }

  public static Pose2d shooterPose(Supplier<Pose2d> drivePose) {
    return drivePose
        .get()
        .transformBy(
            new Transform2d(
                Units.inchesToMeters(11.0), drivePose.get().getY(), drivePose.get().getRotation()));
  }

  /** The periodic function */
  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterIOinputsAutoLogged);
    Logger.processInputs("Shooter/Flywheel", shooterIOinputsAutoLogged);

    feederIO.updateInputs(feederIOInputsAutoLogged);
    Logger.processInputs("Shooter/Feeder", feederIOInputsAutoLogged);
  }
}

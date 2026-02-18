// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim shooter;
  private final SimpleMotorFeedforward shooterFF;
  private final BangBangController bbController;
  private double appliedVoltage = 0.0;

  public ShooterIOSim() {

    shooter =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.Mechanical.J.in(KilogramSquareMeters),
                ShooterConstants.Mechanical.shooterWheelGearRatio),
            DCMotor.getKrakenX60Foc(1));
    shooterFF =
        new SimpleMotorFeedforward(
            ShooterConstants.Tuning.kS, ShooterConstants.Tuning.kV, ShooterConstants.Tuning.kA);
    bbController = new BangBangController();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    shooter.update(0.02);

    appliedVoltage =
        bbController.calculate(shooter.getAngularVelocityRadPerSec())
                * RobotController.getBatteryVoltage()
            + (shooterFF.calculate(shooter.getAngularVelocityRadPerSec()));

    shooter.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
    Logger.recordOutput("DEBUG/AppliedVoltage", appliedVoltage);
    inputs.velocity = shooter.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = shooter.getInputVoltage();
    inputs.statorCurrent = shooter.getCurrentDrawAmps();
    inputs.positionRadPerSec = 0.0;
    inputs.setpoint = bbController.getSetpoint();
  }

  @Override
  public void setVoltage(double volts) {
    shooter.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(AngularVelocity velocityRadPerSec) {
    bbController.setSetpoint(velocityRadPerSec.in(RadiansPerSecond));
  }
}

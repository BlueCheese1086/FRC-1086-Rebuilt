// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.Tuning.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim shooter;
  private final SimpleMotorFeedforward shooterFF;
  private final PIDController pidController;
  private double appliedVoltage = 0.0;

  public ShooterIOSim() {
    shooter =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.Mechanical.J.in(KilogramSquareMeters),
                ShooterConstants.Mechanical.shooterWheelGearRatio),
            DCMotor.getKrakenX60Foc(1));
    shooterFF = new SimpleMotorFeedforward(0.0, 0.019, 0.0);
    pidController = new PIDController(0.01, 0.0, 0.0);
    pidController.setTolerance(30.0);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {

    shooter.update(0.02);

    appliedVoltage =
        (pidController.calculate(shooter.getAngularVelocityRadPerSec())
                * RobotController.getBatteryVoltage())
            + (shooterFF.calculate(shooter.getAngularVelocityRadPerSec()));

    shooter.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
    inputs.velocity = shooter.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = shooter.getInputVoltage();
    inputs.statorCurrent = shooter.getCurrentDrawAmps();
    inputs.positionRadPerSec = 0.0;
    inputs.setpoint = pidController.getSetpoint();
    inputs.atSetpoint = pidController.atSetpoint();

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kd.hasChanged(hashCode())
        || kv.hasChanged(hashCode())
        || ka.hasChanged(hashCode())
        || ks.hasChanged(hashCode())) {
      updateClosedLoop();
    }
  }

  private void updateClosedLoop() {
    shooterFF.setKa(ka.getAsDouble());
    shooterFF.setKv(kv.getAsDouble());
    shooterFF.setKs(ks.getAsDouble());
    pidController.setP(kP.getAsDouble());
    pidController.setI(kI.getAsDouble());
    pidController.setD(kd.getAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    shooter.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(AngularVelocity velocityRadPerSec) {
    pidController.setSetpoint(velocityRadPerSec.in(RadiansPerSecond));
  }
}

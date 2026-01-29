// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.InputMismatchException;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim shooter;
    private final SimpleMotorFeedforward shooterFF;
    private final BangBangController bbController;
    private final PIDController pid = new PIDController(0.12, 0, 0);

    public ShooterIOSim() {

        shooter = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.Mechanical.J.in(KilogramSquareMeters),
                ShooterConstants.Mechanical.gearing), DCMotor.getKrakenX60Foc(1));
        shooterFF = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV,
                ShooterConstants.PID.kA);
        bbController = new BangBangController(15.0);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        shooter.update(0.02);
        shooter.setInputVoltage(bbController.calculate(shooter.getAngularVelocityRadPerSec()) * 12.0 +
               + shooterFF.calculate(bbController.getSetpoint()));
        // shooter.setInputVoltage(pid.calculate(shooter.getAngularVelocityRadPerSec()));
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
    public void setVelocity(double velocity) {
        pid.setSetpoint(velocity);
        bbController.setSetpoint(velocity);
    }
}

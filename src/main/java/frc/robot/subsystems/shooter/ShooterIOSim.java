// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim left, middle, right;
    private final PIDController left_pid, middle_pid, right_pid;
    private final SimpleMotorFeedforward left_ff, middle_ff, right_ff;

    private Voltage appliedVoltage;

    public ShooterIOSim() {
        appliedVoltage = Volts.zero();

        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.Mechanical.J.in(KilogramSquareMeters),
                ShooterConstants.Mechanical.gearing);

        left = new FlywheelSim(plant, DCMotor.getKrakenX60Foc(1));
        middle = new FlywheelSim(plant, DCMotor.getKrakenX60Foc(1));
        right = new FlywheelSim(plant, DCMotor.getKrakenX60Foc(1));

        left_pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
        middle_pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
        right_pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);

        left_ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
        middle_ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV,
                ShooterConstants.PID.kA);
        right_ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV,
                ShooterConstants.PID.kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        left.setInputVoltage(left_pid.calculate(left.getAngularVelocityRadPerSec()) + left_ff.calculate(left_pid.getSetpoint()));
        middle.setInputVoltage(middle_pid.calculate(left.getAngularVelocityRadPerSec()) + middle_ff.calculate(middle_pid.getSetpoint()));
        right.setInputVoltage(right_pid.calculate(left.getAngularVelocityRadPerSec()) + right_ff.calculate(right_pid.getSetpoint()));

        left.update(0.02);
        middle.update(0.02);
        right.update(0.02);

        inputs.velocity = right.getAngularVelocityRadPerSec();

    }

    @Override
    public void setVoltage(double volts) {
        left.setInputVoltage(volts);
        middle.setInputVoltage(volts);
        right.setInputVoltage(volts);
    }

    @Override
    public void setVelocity(double velocity) {
        left_pid.setSetpoint(velocity);
        middle_pid.setSetpoint(velocity);
        right_pid.setSetpoint(velocity);
    }
}

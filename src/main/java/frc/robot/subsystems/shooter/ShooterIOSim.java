// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim left, middle, right;
    private final PIDController left_pid, middle_pid, right_pid;
    private final SimpleMotorFeedforward left_ff, middle_ff, right_ff;

    private double targetRPS;
    private double manualVolts;
    private boolean isVelocityControl;

    public ShooterIOSim() {
        LinearSystem<N1,N1,N1> plant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0008, 1.0);

        left = new FlywheelSim(plant, DCMotor.getKrakenX60Foc(1));
        middle = new FlywheelSim(plant, DCMotor.getKrakenX60Foc(1));
        right = new FlywheelSim(plant, DCMotor.getKrakenX60Foc(1));

        left_pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
        middle_pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
        right_pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);

        left_ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
        middle_ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
        right_ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        if (isVelocityControl) {
            double leftVolts = left_pid.calculate(left.getAngularVelocityRPM() / 60.0, targetRPS) + left_ff.calculate(targetRPS);
            double middleVolts = middle_pid.calculate(middle.getAngularVelocityRPM() / 60.0, targetRPS) + middle_ff.calculate(targetRPS);
            double rightVolts = right_pid.calculate(right.getAngularVelocityRPM() / 60.0, targetRPS) + right_ff.calculate(targetRPS);
            
            left.setInputVoltage(leftVolts);
            middle.setInputVoltage(middleVolts);
            right.setInputVoltage(rightVolts);
        } else {
            left.setInputVoltage(manualVolts);
            middle.setInputVoltage(manualVolts);
            right.setInputVoltage(manualVolts);
        }
        
        left.update(0.02);
        middle.update(0.02);
        right.update(0.02);

        inputs.leftCurrentRPM = left.getAngularVelocityRPM();
        inputs.middleCurrentRPM = middle.getAngularVelocityRPM();
        inputs.rightCurrentRPM = right.getAngularVelocityRPM();
    }

    @Override
    public void setVolts(double volts) {
        isVelocityControl = false;
        manualVolts = volts;
    }

    @Override
    public void setRPM(double rpm) {
        isVelocityControl = true;
        targetRPS = rpm / 60.0;
    }
}

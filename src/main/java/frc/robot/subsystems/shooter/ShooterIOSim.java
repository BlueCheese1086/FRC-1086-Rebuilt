// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim left;
    private final FlywheelSim middle;
    private final FlywheelSim right;

    private final PIDController pid;
    private final SimpleMotorFeedforward ff;

    private double appliedVoltage = 0.0;

    public ShooterIOSim() {
        left = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0008, 1.0),
            DCMotor.getKrakenX60Foc(1));
        middle = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0008, 1.0),
            DCMotor.getKrakenX60Foc(1));
        right = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0008, 1.0),
            DCMotor.getKrakenX60Foc(1));

        pid = new PIDController(ShooterConstants.PID.kP, ShooterConstants.PID.kI, ShooterConstants.PID.kD);
        ff = new SimpleMotorFeedforward(ShooterConstants.PID.kS, ShooterConstants.PID.kV, ShooterConstants.PID.kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        
    }

    @Override
    public void setVolts(double volts) {
        
    }

    @Override
    public void setRPM(double rpm) {
        
    }
}

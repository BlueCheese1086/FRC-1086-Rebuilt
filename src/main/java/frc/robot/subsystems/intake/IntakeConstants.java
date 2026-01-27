// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class IntakeConstants {
    public static class PID {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kG = 0.0;
    }

    public static class Mechanical { // TODO: Update all of these with the actual values
        public static final Distance intakeLength = Inches.of(14);
        public static final double gearing = 1.0;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.measure.MomentOfInertia;

/** Add your docs here. */
public class ShooterConstants {
    public static class PID {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;
        public static final double kV = 0.1;
        public static final double kA = 0.0;
    }

    public static class Mechanical {
        public static final MomentOfInertia J = KilogramSquareMeters.of(0.001);
        public static final double gearing = 1.0;
    }
}

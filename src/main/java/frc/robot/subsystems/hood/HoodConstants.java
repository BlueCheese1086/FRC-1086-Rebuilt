// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class HoodConstants {
    public static class Mechanical {
        public static final Distance kServoLength = Millimeters.of(100);
        public static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
        public static final double kPositionTolerance = 0.01;
        public static final double minPosition = 0.01;
        public static final double maxPosition = 0.77;
    }
}

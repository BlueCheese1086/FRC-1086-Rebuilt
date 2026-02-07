// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Per;

/** Add your docs here. */
public class ClimbConstants {
    public static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));

    public enum Position {
        HOMED(0),
        EXTEND_HOPPER(2),
        HANGING(6),
        HUNG(0.2);

        public final double inches;

        private Position(double inches) {
            this.inches = inches;
        }

        public Angle motorAngle() {
            final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
            return Rotations.of(angleMeasure.in(Rotations)); 
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class HoodConstants {
  public static class Mechanical {
    public static final Distance kServoLength = Millimeters.of(100);
    public static final Distance kDesiredLength = Millimeters.of(100);
    public static final double scaledDist =
        MathUtil.clamp(kDesiredLength.in(Millimeters) / kServoLength.in(Millimeters), 0, 1.0);
    public static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    public static final double kPositionTolerance = 0.01;
    public static final double minPosition = 0.01;
    public static final double maxPosition = 0.77;
    // do we need the ratio? idk im genuinely asking
  }

  public static class Targeting {
    public static final double minAngleDeg = 54.0;
    public static final double maxAngleDeg = 81.0;
  }

  public static class Setpoints {
    public static final Angle passAngle = Degrees.of(60.0);
  }
}

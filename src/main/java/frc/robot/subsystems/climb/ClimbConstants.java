// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class ClimbConstants {
  public static final int climbID = 0;

  public static final double kG = 1.0;
  public static final double kS = 0.0;

  public static final double kV = 12.0 / 6000.0; // max battery voltage / max motor rpm
  public static final double kA = 0.5;
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.5;

  public static final double gearing = 44.44444;
  public static final double radius = Units.inchesToMeters(0.265466625359);

  public static final InvertedValue invertedValue = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

  private static final double maxHeight = 10.0;

  public static final double extendedHeight = maxHeight;
  public static final double retractedHeight = 0.0;
  public static final double SysIdTolerance = 0.01;

  public static class Setpoints {
    public static final double climbExtend = Units.inchesToMeters(8.515); // parallel from tube
    public static final double climbScore = Units.inchesToMeters(3.860572); // should be retracted
    public static final double retracted =
        Units.inchesToMeters(
            3.860572); // in cad this is the parallel distance between the tube and the top of the
    // hook, if it's supposed to be 0 and you're reading this go ahead and change
    // it,
    // TODO: Change setpoints so that it actually moves please
  }

  public static final double thesamethingasthevalueinthefeedback = 2 * Math.PI * radius * gearing;

  public static final AngularVelocity krackenFreeSpeed = RotationsPerSecond.of(6000.0 * 60);
}

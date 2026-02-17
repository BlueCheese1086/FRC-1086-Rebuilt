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
  public static final double kG = 1.0;
  public static final double kS = 0.0;

  public static final double kV = 12.0 / 6000.0; // max battery voltage / max motor rpm
  public static final double kA = 0.5;
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.5;

  public static class Setpoints {
    public static final double raised = 10.0; // The point where the 
    public static final double holding = 2.0; // Find the Raised point where the climber can lift high enough 
    public static final double score = 2.0; // Find the Raised point where the climber can score
  }

  public static final double gearing =
      10.0; // TODO: FIX THIS ASAP! talk to james abt cad next pcs meeting or smth idk

  public static final InvertedValue invertedValue = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

  public static final double radius = Units.inchesToMeters(0.265466625359);

  private static final double maxHeight = 10.0;

  public static final double extendedHeight = maxHeight;
  public static final double retractedHeight = 0.0;

  public static final AngularVelocity krakenFreeSpeed = RotationsPerSecond.of(6000.0 / 60);
}
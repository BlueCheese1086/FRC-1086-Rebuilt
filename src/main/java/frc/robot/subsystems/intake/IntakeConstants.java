// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeConstants {
  public static class PID {
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public static class CurrentLimits {
    public static final Current maxSupply = Amps.of(60);
    public static final Current maxStator = Amps.of(60.0);
  }

  public static class VoltageLimits {
    public static final Voltage peakForwardVoltage = Volts.of(12);
    public static final Voltage peakReverseVoltage = Volts.of(-12);
  }

  public static class Setpoints {
    public static final Angle stowed = Radians.of(1.744);
    public static final Angle homed = Degrees.of(110);
    public static final Angle agitate = Degrees.of(20);
    public static final Angle deployed = Radians.of(-0.122);

    public static final Voltage run = Volts.of(3);
  }

  public static class Mechanical { // TODO: Update all of these with the actual values
    public static final Distance intakeLength = Inches.of(14);
    public static final double gearing = 50.0;
    public static final Angle kPositionTolerance = Radians.of(0.1);
  }
}

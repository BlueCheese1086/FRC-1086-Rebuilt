// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class IntakeConstants {
  public static class PID {
    public static LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/PID/kP", 56.408);
    public static LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/PID/kI", 0.0);
    public static LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/PID/kD", 16.039);

    public static LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/FF/kS", 0.42893);
    public static LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/FF/kG", 0.3871);
    public static LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/FF/kV", 2.7997);
    public static LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/FF/kA", 2.7468);
  }

  public static class CurrentLimits {
    public static final Current maxSupply = Amps.of(50);
    public static final Current maxStator = Amps.of(40.0);
  }

  public static class VoltageLimits {
    public static final Voltage peakForwardVoltage =
        Volts.of(12); // is this rollers or is this pivot, both - martin
    public static final Voltage peakReverseVoltage = Volts.of(-12);
  }

  public static class Setpoints {
    public static final Angle stowed = Degrees.of(115.0);
    public static final Angle homed = Degrees.of(110.0);
    public static final Angle agitate = Degrees.of(20.0);
    public static final Angle deployed = Degrees.of(4.0);
    // im js letting yall know i might have a funny idea for agitation while shooting so js be
    // prepared for that
    // i wanna try slowly moving intake up to homed or stowed while shooting cuz pumping will toss
    // but not compress them towards shooter that well
    public static final Voltage run = Volts.of(12.0);
  }

  public static class Mechanical { // TODO: Update all of these with the actual values
    public static final Distance intakeLength = Inches.of(14);
    public static final double gearing = 50.0;
    public static final Angle kPositionTolerance = Degrees.of(1.0);
    // all of these are in cad right
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ShooterConstants {
  public static class Mechanical {
    public static final double gearing = 1.0;
    public static final Distance diameter = Inches.of(4.125);
    public static final AngularVelocity idleVelocity = RadiansPerSecond.of(10);
  }

  public static class CurrentLimits {
    public static final Current maxSupply = Amps.of(60);
    public static final Current maxStator = Amps.of(80);
  }

  public static class VoltageLimits {
    public static final Voltage minVoltage = Volts.of(-12.0);
    public static final Voltage maxVoltage = Volts.of(12.0);
  }

  public static class PID {
    public static LoggedTunableNumber kP =
        new LoggedTunableNumber("Shooter/PID/kP", Preferences.getDouble("Shooter_kP", 9.0));
    public static LoggedTunableNumber kI =
        new LoggedTunableNumber("Shooter/PID/kI", Preferences.getDouble("Shooter_kI", 0));
    public static LoggedTunableNumber kD =
        new LoggedTunableNumber("Shooter/PID/kD", Preferences.getDouble("Shooter_kD", 0));
    public static LoggedTunableNumber kS =
        new LoggedTunableNumber("Shooter/PID/kS", Preferences.getDouble("Shooter_kS", 0));
    public static LoggedTunableNumber kG =
        new LoggedTunableNumber("Shooter/PID/kG", Preferences.getDouble("Shooter_kG", 0));
    public static LoggedTunableNumber kV =
        new LoggedTunableNumber("Shooter/PID/kV", Preferences.getDouble("Shooter_kV", 10));
    public static LoggedTunableNumber kA =
        new LoggedTunableNumber("Shooter/PID/kA", Preferences.getDouble("Shooter_kA", 0));
  }
}

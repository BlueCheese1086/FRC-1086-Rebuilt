// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class IntakeConstants {
  public static class Mechanical {
    public static final double rollerGearing = 1.0;
    public static final double pivotGearing = 33.0 + (1.0 / 3.0);

    public static final boolean pivotInverted = true;
    public static final boolean rollerInverted = false;

    public static final Angle maxAngle = Degrees.of(135);
  }

  public static class Pivot {

    public static final Angle deploy = Degrees.zero();
    public static final Angle retracted =
        Degrees.of(123); // Temporary please find actual number in real life.

    public static class CurrentLimits {
      public static final Current maxSupply = Amps.of(40);
      public static final Current maxStator = Amps.of(30);
    }

    public static class VoltageLimits {
      public static final Voltage maxForward = Volts.of(12);
      public static final Voltage maxReverse = Volts.of(-12);
    }

    public static class PID {
      public static final LoggedTunableNumber kP =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kP", Preferences.getDouble("Intake_Pivot_kP", 12.5));
      public static final LoggedTunableNumber kI =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kI", Preferences.getDouble("Intake_Pivot_kI", 0));
      public static final LoggedTunableNumber kD =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kD", Preferences.getDouble("Intake_Pivot_kD", 0));
      public static final LoggedTunableNumber kS =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kS", Preferences.getDouble("Intake_Pivot_kS", 0.2383));
      public static final LoggedTunableNumber kG =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kG", Preferences.getDouble("Intake_Pivot_kG", 0.143));
      public static final LoggedTunableNumber kV =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kV", Preferences.getDouble("Intake_Pivot_kV", 4.9277));
      public static final LoggedTunableNumber kA =
          new LoggedTunableNumber(
              "Intake/Pivot/PID/kA", Preferences.getDouble("Intake_Pivot_kA", 0));
    }

    public static class Output {
      public static final double maxForward = 1.0;
      public static final double maxReverse = -1.0;
    }
  }

  public static class Roller {
    public static class CurrentLimits {
      public static final Current maxSupply = Amps.of(40);
      public static final Current maxStator = Amps.of(60);
    }

    public static final Voltage running = Volts.of(12.0);

    public static class VoltageLimits {
      public static final Voltage maxForward = Volts.of(12);
      public static final Voltage maxReverse = Volts.of(-12);
    }

    public static class PID {
      public static final LoggedTunableNumber kP =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kP", Preferences.getDouble("Intake_Roller_kP", 0));
      public static final LoggedTunableNumber kI =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kI", Preferences.getDouble("Intake_Roller_kI", 0));
      public static final LoggedTunableNumber kD =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kD", Preferences.getDouble("Intake_Roller_kD", 0));
      public static final LoggedTunableNumber kS =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kS", Preferences.getDouble("Intake_Roller_kS", 0));
      public static final LoggedTunableNumber kG =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kG", Preferences.getDouble("Intake_Roller_kG", 0));
      public static final LoggedTunableNumber kV =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kV", Preferences.getDouble("Intake_Roller_kV", 0));
      public static final LoggedTunableNumber kA =
          new LoggedTunableNumber(
              "Intake/Roller/PID/kA", Preferences.getDouble("Intake_Roller_kA", 0));
    }

    public static class Output {
      public static final double maxForward = 1.0;
      public static final double maxReverse = -1.0;
    }
  }
}

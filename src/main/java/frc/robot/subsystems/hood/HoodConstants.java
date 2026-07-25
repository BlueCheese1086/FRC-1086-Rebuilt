// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class HoodConstants {

  public static LoggedTunableNumber hoodAngle = new LoggedTunableNumber("Manual Hood", 0.0);

  public static class Mechanical {
    public static final double gearing = 226.5;
    public static final double minAngle = 0.0;
    public static final double travel = 29;
    public static final double maxAngle = minAngle + travel;
  }

  public static class CurrentLimits {
    public static final Current maxSupply = Amps.of(30);
    public static final Current maxStator = Amps.of(60);
  }

  public static class VoltageLimits {
    public static final Voltage minVoltage = Volts.of(-12.0);
    public static final Voltage maxVoltage = Volts.of(12.0);
  }

  public static class PID {
    public static LoggedTunableNumber kP =
        new LoggedTunableNumber("Hood/PID/kP", Preferences.getDouble("Hood_kP", 200.0));
    public static LoggedTunableNumber kI =
        new LoggedTunableNumber("Hood/PID/kI", Preferences.getDouble("Hood_kI", 0));
    public static LoggedTunableNumber kD =
        new LoggedTunableNumber("Hood/PID/kD", Preferences.getDouble("Hood_kD", 0));
    public static LoggedTunableNumber kS =
        new LoggedTunableNumber("Hood/PID/kS", Preferences.getDouble("Hood_kS", 0.23046875));
    public static LoggedTunableNumber kG =
        new LoggedTunableNumber("Hood/PID/kG", Preferences.getDouble("Hood_kG", 0));
    public static LoggedTunableNumber kV =
        new LoggedTunableNumber("Hood/PID/kV", Preferences.getDouble("Hood_kV", 24));
    public static LoggedTunableNumber kA =
        new LoggedTunableNumber(
            "Hood/PID/kA", Preferences.getDouble("Hood_kA", 1.7207000255584717));
  }
}

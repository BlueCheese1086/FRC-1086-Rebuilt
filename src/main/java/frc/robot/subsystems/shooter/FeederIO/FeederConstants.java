// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.FeederIO;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class FeederConstants {
  public static class Mechanical {
    public static final double gearing = 1.0;
    public static final boolean inverted = false;
  }

  public static class CurrentLimits {
    public static final Current maxSupply = Amps.of(60);
    public static final Current maxStator = Amps.of(80);
  }

  public static final Voltage running = Volts.of(12.0);
  public static final Current torqueRun = Amps.of(80.0);

  public static class VoltageLimits {
    public static final Voltage maxForward = Volts.of(16);
    public static final Voltage maxReverse = Volts.of(-16);
  }
}

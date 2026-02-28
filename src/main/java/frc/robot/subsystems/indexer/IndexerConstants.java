// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IndexerConstants {
  public static class CurrentLimits {
    public static final Current maxSupply = Amps.of(40);
    public static final Current maxStator = Amps.of(40.0); // a bit high imo
  }

  public static class VoltageLimits {
    public static final Voltage peakForwardVoltage = Volts.of(12);
    public static final Voltage peakReverseVoltage = Volts.of(-12);
  }

  public static class Setpoints {
    public static final Voltage intake = Volts.of(3);
    public static final Voltage feed = Volts.of(12.0);
  }
}
// should we add the reduction that indexer has in cad? it's a 3:1 reduction, nah it is fine because
// it is a voltage based subsystem

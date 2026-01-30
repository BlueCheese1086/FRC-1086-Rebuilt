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
        public static final Current maxSupply = Amps.of(60);
        public static final Current maxStator = Amps.of(60.0);
    }

    public static class VoltageLimits {
        public static final Voltage peakForwardVoltage = Volts.of(12);
        public static final Voltage peakReverseVoltage = Volts.of(-12);
    }
}

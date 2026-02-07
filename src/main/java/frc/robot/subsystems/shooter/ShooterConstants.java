// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.measure.MomentOfInertia;

/** Add your docs here. */
public class ShooterConstants {
  public static class Tuning {
    public static final double kS = 0.1;
    public static final double kV = 12.0 / 100.0; // adjust later when i feel like it
    public static final double kA = 0.0;

    public static final double cruiseVelocity = 5729.58 / 60; // ~ 600 rad per sec;
    public static final double acceleration =
        cruiseVelocity
            / 0.23; // Cruise velocity / spin up time estimated, low numbers equal more brownouts,
    // and high numbers more stable.
  }

  public static class Mechanical {
    public static final MomentOfInertia J = KilogramSquareMeters.of(0.001);
    public static final double gearing = 1.0;
  }
}

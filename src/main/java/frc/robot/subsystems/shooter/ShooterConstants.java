// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class ShooterConstants {
  public static class Tuning {
    public static final double kS = 0.0;
    public static final double kV =
        12.0
            / RadiansPerSecond.of(DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec)
                .in(RotationsPerSecond); // 113.067;
    public static final double kA = 0.0;

    public static final double cruiseVelocity = 6000.0 / 60.0; // ~ 600 rad per sec;
    public static final double acceleration =
        cruiseVelocity
            / 0.23; // Cruise velocity / spin up time estimated, low numbers equal more brownouts,
    // and high numbers more stable.
    public static final LoggedNetworkNumber velocitySetpoint =
        new LoggedNetworkNumber("/Tuning/Velocity Setpoint", 100.0);
    public static final LoggedNetworkNumber voltageSetpoint =
        new LoggedNetworkNumber("/Tuning/Voltage Setpoint", 4.5);
  }
  // i could be wrong but do we need a current limit bc id like to not brown out while sotm if
  // possible
  // i could be wrong but do we need a current limit bc id like to not brown out while sotm if
  // possible
  public static class Targeting {
    public static final double minRpm = 1500.0;
    public static final double maxRpm = 6000.0;
    public static final double stationaryRpm = 3200.0;
    public static final double movingSpeedThresholdMps = 0.25;
    public static final double movingRpmChangeWeight = 2.0;
    public static final double movingHoodChangeWeight = 0.5;
    public static final String FileName = "Shot.csv";
  }

  public static class FeederSetpoints {
    public static final Voltage run = Volts.of(12);
  }

  public static class Mechanical {
    public static final MomentOfInertia J = KilogramSquareMeters.of(0.001);
    public static final Distance shooterHeight = Inches.of(24.5);
    public static final Distance flywheelRadius = Inches.of(2.0);
    public static final double shooterWheelGearRatio = 1.0;
    public static final Distance shooterXOffset = Inches.of(-10.0);
    public static final Distance shooterYOffset = Inches.of(0.0);

    // The position of the shooter relative to the robot's center, used for
    // calculating the distance to the hub.
    public static final Transform3d shooterPose =
        new Transform3d(
            shooterXOffset.in(Meters),
            shooterYOffset.in(Meters),
            shooterHeight.in(Meters),
            new Rotation3d());
  }

  public static class ShooterTransforms {
    public static final Transform3d leftShooter =
        new Transform3d(new Translation3d(Units.inchesToMeters(0), 0, 0), Rotation3d.kZero);
    public static final Transform3d centerShooter = new Transform3d();
    public static final Transform3d rightShooter = new Transform3d();
  }
}

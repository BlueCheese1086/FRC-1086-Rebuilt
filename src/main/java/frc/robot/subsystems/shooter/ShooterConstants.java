// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class ShooterConstants {
  public static class Tuning {
    public static final LoggedTunableNumber leftkv =
        new LoggedTunableNumber("/Shooter/Left/Kv", 0.125);
    public static final LoggedTunableNumber leftks =
        new LoggedTunableNumber("/Shooter/Left/Ks", 0.14819);
    public static final LoggedTunableNumber leftka =
        new LoggedTunableNumber("/Shooter/Left/ka", 0.0024824);
    public static final LoggedTunableNumber leftkP =
        new LoggedTunableNumber("/Shooter/Left/kP", 0.029853);
    public static final LoggedTunableNumber rightkv =
        new LoggedTunableNumber("/Shooter/Right/Kv", 0.125);
    public static final LoggedTunableNumber rightks =
        new LoggedTunableNumber("/Shooter/Right/Ks", 0.14819);
    public static final LoggedTunableNumber rightka =
        new LoggedTunableNumber("/Shooter/Right/ka", 0.0024824);
    public static final LoggedTunableNumber rightkP =
        new LoggedTunableNumber("/Shooter/Right/kP", 0.029853);
    public static final LoggedTunableNumber middlekv =
        new LoggedTunableNumber("/Shooter/Middle/Kv", 0.125);
    public static final LoggedTunableNumber middleks =
        new LoggedTunableNumber("/Shooter/Middle/Ks", 0.14819);
    public static final LoggedTunableNumber middleka =
        new LoggedTunableNumber("/Shooter/Middle/ka", 0.0024824);
    public static final LoggedTunableNumber middlekP =
        new LoggedTunableNumber("/Shooter/Middle/kP", 0.029853);

    public static final LoggedNetworkNumber velocitySetpoint =
        new LoggedNetworkNumber("/Tuning/Velocity Setpoint", 350.0);
    public static final LoggedNetworkNumber voltageSetpoint =
        new LoggedNetworkNumber("/Tuning/Voltage Setpoint", 4.5);

    public static final Slot0Configs leftShooterConfigs =
        new Slot0Configs()
            .withKP(leftkP.getAsDouble())
            .withKV(leftkv.getAsDouble())
            .withKS(leftks.getAsDouble())
            .withKA(leftka.getAsDouble());
    public static final Slot0Configs middleShooterConfigs =
        new Slot0Configs()
            .withKP(middlekP.getAsDouble())
            .withKV(middlekv.getAsDouble())
            .withKS(middleks.getAsDouble())
            .withKA(middleka.getAsDouble());
    public static final Slot0Configs rightShooterConfigs =
        new Slot0Configs()
            .withKP(rightkP.getAsDouble())
            .withKV(rightkv.getAsDouble())
            .withKS(rightks.getAsDouble())
            .withKA(rightka.getAsDouble());
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
    public static final String FileName = "/home/lvuser/logs/Shot.csv";
  }

  public static class FeederSetpoints {
    public static final Voltage run = Volts.of(8);
  }

  public static class Mechanical {
    public static final MomentOfInertia J = KilogramSquareMeters.of(0.001);
    public static final Distance shooterHeight = Inches.of(26.0);
    public static final Distance flywheelRadius = Inches.of(2.0);
    public static final double shooterWheelGearRatio = 1.0;
    public static final Distance shooterXOffset = Inches.of(-10.0);
    public static final Distance shooterYOffset = Inches.of(0.0);

    // The position of the shooter relative to the robot's center, used for
    // calculating the distance to the hub.
    public static final Transform3d shooterPose =
        new Transform3d(
            -Units.inchesToMeters(8.5),
            shooterYOffset.in(Meters),
            shooterHeight.in(Meters),
            new Rotation3d(0.0, 0.0, 0.0));
    public static Transform3d robotToLauncher =
        new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));
  }

  public static class ShooterTransforms {
    public static final Transform3d leftShooter =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(8.5), Units.inchesToMeters(6.5), Units.inchesToMeters(26)),
            Rotation3d.kZero);
    public static final Transform3d centerShooter =
        new Transform3d(
            -Units.inchesToMeters(8.5),
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(26),
            Rotation3d.kZero);
    public static final Transform3d rightShooter =
        new Transform3d(
            -Units.inchesToMeters(8.5),
            -Units.inchesToMeters(6.5),
            Units.inchesToMeters(26),
            Rotation3d.kZero);
  }
}

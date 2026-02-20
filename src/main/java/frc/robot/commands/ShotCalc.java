// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Add your docs here. */
public class ShotCalc {
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

  static {
    distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
    distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
    distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
  }

  public static Shot getShot(Distance distance) {
    return distanceToShotMap.get(distance);
  }

  public static double getShotMPS(Distance distance) {
    return distanceToShotMap.get(distance).getMPS();
  }

  public static class Shot {
    public final double shooterRPM;
    public final double hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      this.shooterRPM = shooterRPM;
      this.hoodPosition = hoodPosition;
    }

    public double getMPS() {
      return Rotation2d.fromDegrees(Hood.AngleToPosition.get(hoodPosition)).getCos()
          * (Units.rotationsToRadians(shooterRPM / 60)
              * ShooterConstants.Mechanical.flywheelRadius.in(Meters));
    }
  }

  public static double[] getTargetVelocities(double positionMeters) {
    return new double[] {};
  }
}

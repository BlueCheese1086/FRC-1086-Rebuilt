// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
    distanceToShotMap.put(Meters.of(1.4478), new Shot(3351.803102, 78));
    distanceToShotMap.put(Meters.of(1.8), new Shot(3151.267873, 65));
    distanceToShotMap.put(Meters.of(2.054), new Shot(3151.267873, 75));
    distanceToShotMap.put(Meters.of(3.048), new Shot(3437.746771, 65));
    distanceToShotMap.put(Meters.of(5.334), new Shot(3819.718634, 55));
    // addShotParams(1.4478, 3351.803102, 78.0);
    // addShotParams(1.8, 3151.267873, 65.0);
    // addShotParams(2.054, 3151.267873, 75.0);
    // addShotParams(3.048, 3437.746771, 65.0);
    // addShotParams(5.334, 3819.718634, 55.0)

    // table
    // Dist, RPM, Angle, Pose
    // 2.430, 350, 68.274, (2.288, 4.753, -17.44)
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
      return Rotation2d.fromDegrees(Hood.InvertAngleToPosition.get(hoodPosition)).getCos()
          * (Units.rotationsToRadians(shooterRPM / 60)
              * ShooterConstants.Mechanical.flywheelRadius.in(Meters));
    }

    public Angle getAngle() {
      return Degrees.of(hoodPosition);
    }
  }

  public static double[] getTargetVelocities(double positionMeters) {
    return new double[] {};
  }
}

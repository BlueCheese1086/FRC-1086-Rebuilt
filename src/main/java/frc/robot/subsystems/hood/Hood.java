// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public static final InterpolatingDoubleTreeMap AngleToPosition = new InterpolatingDoubleTreeMap();

  public static final InterpolatingDoubleTreeMap InvertAngleToPosition =
      new InterpolatingDoubleTreeMap();
  private Angle setAngle = Radians.zero();
  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

  static {
    AngleToPosition.put(81.0, 0.01);
    AngleToPosition.put(54.0, 0.77);

    InvertAngleToPosition.put(0.01, 81.0);
    InvertAngleToPosition.put(0.77, 54.0);
  }

  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public Command setAngle(Angle angle) {
    Logger.recordOutput("/Hood/Map/0-1", AngleToPosition.get(angle.in(Degrees)));
    Logger.recordOutput("/Hood/Map/key", angle.in(Degrees));
    return this.run(() -> io.setPosition(AngleToPosition.get(angle.in(Degrees))));
  }

  public Command setAngle(Supplier<Angle> angle) {
    Logger.recordOutput("/Hood/Map/0-1", AngleToPosition.get(angle.get().in(Degrees)));
    Logger.recordOutput("/Hood/Map/key", angle.get().in(Degrees));
    return this.run(() -> io.setPosition(AngleToPosition.get(angle.get().in(Degrees))))
        .withTimeout(0.1);
  }

  /* expects a value between 0 and 1 */
  public void setPosition(DoubleSupplier position) {
    Logger.recordOutput("/Hood/Map/0-1", AngleToPosition.get(position.getAsDouble()));
    Logger.recordOutput("/Hood/Map/key", position);
    io.setPosition(AngleToPosition.get(position.getAsDouble()));
  }

  public Command directPWMControl(DoubleSupplier pwm) {
    return this.run(() -> io.setPosition(pwm.getAsDouble())).until(io::atSetpoint);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public Angle getAngle() {
    if (Robot.isSimulation()) {
      return setAngle;
    } else {
      return getAngleFromHoodPos(this.inputs.leftPosition);
    }
  }

  public static Angle getAngleFromHoodPos(double hoodpos) {
    return Degrees.of(AngleToPosition.get(hoodpos));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    inputs.setAngle = setAngle;
    Logger.processInputs("Hood", inputs);
  }
}

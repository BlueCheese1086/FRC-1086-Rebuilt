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
  /** Creates a new Hood. */
  private static final InterpolatingDoubleTreeMap AngleToPosition =
      new InterpolatingDoubleTreeMap();

  private Angle setAngle = Radians.zero();
  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

  static {
    AngleToPosition.put(81.0, 0.01);
    AngleToPosition.put(54.0, 0.77);
  }

  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public Command setAngle(Supplier<Angle> angle) {
    return this.run(
        () -> {
          setAngle = angle.get();
          io.setPosition(AngleToPosition.get(angle.get().in(Degrees)));
        });
  }

  public void setAngle(Angle angle) {
    setAngle = angle;
    io.setPosition(AngleToPosition.get(angle.in(Degrees)));
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          io.setPosition(position.getAsDouble());
        });
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public Angle getAngle() {
    if (Robot.isSimulation()) {
      return setAngle;
    } else {
      return setAngle;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    inputs.setAngle = setAngle;
    Logger.processInputs("Hood", inputs);
  }
}

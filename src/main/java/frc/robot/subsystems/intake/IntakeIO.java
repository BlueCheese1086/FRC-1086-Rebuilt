// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for intake pivot and roller control. */
public interface IntakeIO {
  /** Logged intake sensor and status values. */
  @AutoLog
  public class IntakeInputs {
    /** True when the pivot motor controller is responding. */
    public boolean pivotConnected = false;

    /** Measured pivot angle. */
    public Angle pivotAngle = Radians.zero();

    /** Requested pivot setpoint. */
    public Angle pivotSetpoint = Radians.zero();

    /** Measured pivot angular velocity. */
    public AngularVelocity pivotVelocity = RadiansPerSecond.zero();

    /** True when the pivot is near its requested setpoint. */
    public boolean nearSetpoint = false;

    /** Pivot supply current. */
    public Current pivotSupply = Amps.zero();

    /** Pivot stator current. */
    public Current pivotStator = Amps.zero();

    /** Pivot motor temperature. */
    public Temperature pivotTemp = Celsius.zero();

    /** Pivot applied voltage. */
    public Voltage pivotAppliedVoltage = Volts.zero();

    /** True when the left roller motor controller is responding. */
    public boolean rollerLeftConnected = false;

    /** True when the right roller motor controller is responding. */
    public boolean rollerRightConnected = false;

    /** Measured left roller velocity in radians per second. */
    public double rollerLeftVelocity = 0.0;

    /** Left roller supply current. */
    public Current rollerLeftSupply = Amps.zero();

    /** Left roller stator current. */
    public Current rollerLeftStator = Amps.zero();

    /** Left roller motor temperature. */
    public Temperature rollerLeftTemp = Celsius.zero();

    /** Left roller applied voltage. */
    public Voltage rollerLeftAppliedVoltage = Volts.zero();

    /** Right roller supply current. */
    public Current rollerRightSupply = Amps.zero();

    /** Right roller stator current. */
    public Current rollerRightStator = Amps.zero();

    /** Right roller motor temperature. */
    public Temperature rollerRightTemp = Celsius.zero();

    /** Right roller applied voltage. */
    public Voltage rollerRightAppliedVoltage = Volts.zero();
  }

  /**
   * Updates logged inputs from hardware or simulation.
   *
   * @param inputs mutable input snapshot to fill
   */
  public default void updateInputs(IntakeInputs inputs) {}

  /**
   * Requests a pivot position.
   *
   * @param angle desired pivot angle
   */
  public default void setPosition(Angle angle) {}

  /**
   * Applies roller torque current.
   *
   * @param desired desired current
   */
  public default void setCurrent(Current desired) {}

  /**
   * Applies roller voltage.
   *
   * @param applied desired voltage
   */
  public default void setVoltage(Voltage applied) {}

  /**
   * Applies voltage to one roller for testing.
   *
   * @param applied desired voltage
   * @param left true to drive the left roller, false for the right roller
   */
  public default void setVoltageTest(Voltage applied, boolean left) {}

  /** Switches the IO implementation's control mode, when supported. */
  public default void switchMode() {}

  /**
   * Applies voltage directly to the pivot.
   *
   * @param applied desired pivot voltage
   */
  public default void setPivotVoltage(Voltage applied) {}

  /**
   * Requests a closed-loop roller velocity.
   *
   * @param velocity desired roller velocity
   */
  public default void setRollerVelocity(AngularVelocity velocity) {}
}

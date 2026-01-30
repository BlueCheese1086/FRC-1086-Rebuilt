// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj.LEDPattern;

/** Add your docs here. */
public interface LED_IO {
  public default void setPattern(LEDPattern pattern) {}

  public default void setAnimation(
      ControlRequest animation) {} // Don't put a talonFX control request in here I beg of you.
}

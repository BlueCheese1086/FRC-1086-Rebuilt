// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

/** Add your docs here. */
public class LED_IO_CANdle implements LED_IO {
  private final CANdle candle;

  public LED_IO_CANdle(int id) {
    candle = new CANdle(id);
  }

  @Override
  public void setAnimation(ControlRequest request) {
    candle.setControl(request);
  }
}

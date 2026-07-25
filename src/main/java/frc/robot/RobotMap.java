// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

// TODO: REPLACE WITH ACTUAL NUMBERS
/** Add your docs here. */
public class RobotMap {
  public static final CANBus systemBus = new CANBus("");

  public static class Shooter {
    public static final int UpperLeft = 21;
    public static final int LowerLeft = 23;
    public static final int UpperRight = 22;
    public static final int LowerRight = 24;
  }

  public static final int hood = 61;

  public static final int hopper = 11;
  public static final int feeder = 41;

  public static class Intake {
    public static final int pivot = 31;
    public static final int leftRoller = 32;
    public static final int rightRoller = 33;
  }

  public static class Sensors {
    public static final int leftSensor = 3;
    public static final int rightSensor = 2;
  }
}

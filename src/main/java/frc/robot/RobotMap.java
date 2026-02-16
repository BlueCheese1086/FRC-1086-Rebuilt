// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class RobotMap {
  public static final CANBus driveBus = TunerConstants.kCANBus;
  public static final CANBus systemBus = new CANBus("");

  // TODO: Replace all these with the actual ids
  public static class IntakeMap {
    public static final int pivot = 41;
    public static final int roller = 42;
  }

  public static final int indexer = 11;

  public static class ShooterMap {
    public static final int left = 22;
    public static final int middle = 23;
    public static final int right = 24;
    public static final int feeder = 21;
  }

  public static class HoodMap {
    public static final int left = 0; // tbd
    public static final int right = 0;
  }

  public static final int climber = 31;
}

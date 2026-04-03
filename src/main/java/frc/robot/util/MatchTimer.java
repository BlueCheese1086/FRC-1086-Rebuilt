// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class MatchTimer {
  public static double autoLength = 20.0;
  public static double teleopLength = 140.0;
  private static Timer timer = new Timer();

  public static void reset() {
    timer.reset();
    timer.stop();
  }

  public static void start() {
    timer.start();
  }

  public static double getTime() {
    return (DriverStation.isTeleop() ? teleopLength : autoLength) - timer.get();
  }
}

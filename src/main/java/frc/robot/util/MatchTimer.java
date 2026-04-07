// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class MatchTimer {
  public static double autoLength = 20.0;
  public static double teleopLength = 140.0;
  private static Timer timer = new Timer();
  private static Timer shiftTimer = new Timer();

  public static void reset() {
    timer.reset();
    timer.stop();
    shiftTimer.reset();
    shiftTimer.stop();
  }

  public static void start() {
    timer.start();
    if (DriverStation.isTeleop()) {
      shiftTimer.reset();
      shiftTimer.start();
    }
  }

  public static double getTime() {
    return (DriverStation.isTeleop() ? teleopLength : autoLength) - timer.get();
  }

  public static void periodic() {
    if (MathUtil.isNear(130.0, getTime(), 0.04)
        || MathUtil.isNear(105.0, getTime(), 0.04)
        || MathUtil.isNear(80.0, getTime(), 0.04)
        || MathUtil.isNear(55.0, getTime(), 0.04)
        || MathUtil.isNear(30.0, getTime(), 0.04)) {
      shiftTimer.reset();
      shiftTimer.start();
    }
  }

  public static double getShiftTime() {
    double shiftTotalTime = 30.0;
    if (getTime() > 130) {
      shiftTotalTime = 10.0;
    } else if (getTime() > 30) {
      shiftTotalTime = 25.0;
    } else {
      shiftTotalTime = 30.0;
    }
    return (DriverStation.isAutonomous()
        ? autoLength - timer.get()
        : shiftTotalTime - shiftTimer.get());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

<<<<<<< HEAD
import edu.wpi.first.math.MathUtil;
=======
>>>>>>> 999df29a9d90572172835004b4579ee7f30be876
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class MatchTimer {
  public static double autoLength = 20.0;
  public static double teleopLength = 140.0;
  private static Timer timer = new Timer();
<<<<<<< HEAD
  private static Timer shiftTimer = new Timer();
=======
>>>>>>> 999df29a9d90572172835004b4579ee7f30be876

  public static void reset() {
    timer.reset();
    timer.stop();
<<<<<<< HEAD
    shiftTimer.reset();
    shiftTimer.stop();
=======
>>>>>>> 999df29a9d90572172835004b4579ee7f30be876
  }

  public static void start() {
    timer.start();
<<<<<<< HEAD
    if (DriverStation.isTeleop()) {
      shiftTimer.reset();
      shiftTimer.start();
    }
=======
>>>>>>> 999df29a9d90572172835004b4579ee7f30be876
  }

  public static double getTime() {
    return (DriverStation.isTeleop() ? teleopLength : autoLength) - timer.get();
  }
<<<<<<< HEAD

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
=======
>>>>>>> 999df29a9d90572172835004b4579ee7f30be876
}

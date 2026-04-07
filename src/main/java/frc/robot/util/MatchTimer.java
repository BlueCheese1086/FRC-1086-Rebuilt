// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class MatchTimer {
  public static double autoLength = 20.0;
  public static double teleopLength = 140.0;
  private static Timer timer = new Timer();
  private static Timer shiftTimer = new Timer();

  private static enum Shift {
    Autonomous,
    Transistion,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    EndGame
  }

  public static Shift currentShift = Shift.Autonomous;

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
      currentShift = Shift.Transistion;
    } else {
      currentShift = Shift.Autonomous;
    }
  }

  public static double getTime() {
    return (DriverStation.isTeleop() ? teleopLength : autoLength) - timer.get();
  }

  public static void periodic() {
    if ((MathUtil.isNear(130.0, getTime(), 0.04)
        || MathUtil.isNear(105.0, getTime(), 0.04)
        || MathUtil.isNear(80.0, getTime(), 0.04)
        || MathUtil.isNear(55.0, getTime(), 0.04)
        || MathUtil.isNear(30.0, getTime(), 0.04))) {
      shiftTimer.reset();
      shiftTimer.start();
    }
  }

  public static double getShiftTime() {
    double shiftTotalTime = 30.0;
    if (getTime() > 130) {
      shiftTotalTime = 10.0;
      currentShift = Shift.Transistion;
    } else if (getTime() > 30) {
      shiftTotalTime = 25.0;
      if (MathUtil.isNear(130.0, getTime(), 0.03)) {
        currentShift = Shift.Shift1;
      }
      if (MathUtil.isNear(105.0, getTime(), 0.03)) {
        currentShift = Shift.Shift2;
      }
      if (MathUtil.isNear(80.0, getTime(), 0.03)) {
        currentShift = Shift.Shift3;
      }
      if (MathUtil.isNear(55.0, getTime(), 0.03)) {
        currentShift = Shift.Shift4;
      }
    } else if (DriverStation.isAutonomous()) {
      shiftTotalTime = 20.0;
      currentShift = Shift.Autonomous;
    } else {
      currentShift = Shift.EndGame;
      shiftTotalTime = 30.0;
    }
    return (DriverStation.isAutonomous()
        ? autoLength - timer.get()
        : shiftTotalTime - shiftTimer.get());
  }

  @AutoLogOutput(key = "Field/Hub Active")
  public static boolean isHubActive(Alliance startingAlliance, Alliance currentAlliance) {
    if (DriverStation.isTeleop()) {
      if (MatchTimer.getTime() > 130 || MatchTimer.getTime() < 30) {
        return true;
      } else {
        if (inRange(MatchTimer.getTime(), 105, 130) || inRange(MatchTimer.getTime(), 55, 80)) {
          return startingAlliance.equals(currentAlliance);
        } else {
          return !(startingAlliance.equals(currentAlliance));
        }
      }
    } else {
      return true;
    }
  }

  private static boolean inRange(double value, double min, double max) {
    return value >= min && value <= max;
  }

  public static boolean hubActiveInTof(
      Alliance currentAlliance, Alliance startingAlliance, Pose2d pose) {
    // double timeOfFlight =
    //     LauncherCalculator.getInstance()
    //         .getNaiveTOF(
    //             PoseMath.getDistanceToTarget(
    //                 new Pose3d(pose)
    //                     .transformBy(ShooterConstants.ShooterTransforms.centerShooter)
    //                     .toPose2d(),
    //                 FieldConstants.Hub.hubCenter));
    double timeOfFlight = 2.1;
    boolean activeHub = isHubActive(startingAlliance, currentAlliance);
    if (nearShift(130.0, timeOfFlight)
        || nearShift(105.0, timeOfFlight)
        || nearShift(80.0, timeOfFlight)
        || nearShift(55, timeOfFlight)
        || nearShift(30, timeOfFlight)) {
      if (!activeHub) {
        activeHub = true;
      }
    }
    return activeHub;
  }

  private static boolean nearShift(double shiftStart, double ToF) {
    return inRange(getTime(), shiftStart, shiftStart + ToF);
  }
}

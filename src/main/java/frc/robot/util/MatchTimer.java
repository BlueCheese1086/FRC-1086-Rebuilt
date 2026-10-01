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

/**
 * Tracks match and shift timing for robot logic that needs to react to field-state changes during
 * autonomous and teleoperated play.
 */
public class MatchTimer {
  /** Length, in seconds, of the autonomous period tracked by this utility. */
  public static double autoLength = 20.0;

  /** Length, in seconds, of the teleoperated period tracked by this utility. */
  public static double teleopLength = 140.0;

  private static Timer timer = new Timer();
  private static Timer shiftTimer = new Timer();

  /**
   * Named match segments used to describe the robot's current teleop or autonomous timing state.
   */
  public static enum Shift {
    /** Autonomous period. */
    Autonomous("Auto"),

    /** Initial teleop transition period before the first scoring shift. */
    Transistion("Transistion"),

    /** First main teleop scoring shift. */
    Shift_1("Shift 1"),

    /** Second main teleop scoring shift. */
    Shift_2("Shift 2"),

    /** Third main teleop scoring shift. */
    Shift_3("Shift 3"),

    /** Fourth main teleop scoring shift. */
    Shift_4("Shift 4"),

    /** Final teleop end-game period. */
    End_Game("End Game");

    private String name;

    /**
     * Creates a displayable shift value.
     *
     * @param shiftName human-readable name for the shift
     */
    Shift(String shiftName) {
      this.name = shiftName;
    }

    /**
     * Returns the human-readable name for this shift.
     *
     * @return display name for the shift
     */
    @Override
    public String toString() {
      return name;
    }
  }

  /** Current match segment inferred from the latest timer state. */
  public static Shift currentShift = Shift.Autonomous;

  /** Resets and stops both the overall match timer and the current-shift timer. */
  public static void reset() {
    timer.reset();
    timer.stop();
    shiftTimer.reset();
    shiftTimer.stop();
  }

  /**
   * Starts match timing and initializes the current shift based on the Driver Station mode.
   *
   * <p>Teleop starts with the transition shift timer active. Autonomous starts without resetting
   * the shift timer because autonomous timing is based on the main match timer.
   */
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

  /**
   * Gets the remaining time in the current Driver Station period.
   *
   * @return seconds remaining in autonomous or teleop, depending on Driver Station mode
   */
  public static double getTime() {
    return (DriverStation.isTeleop() ? teleopLength : autoLength) - timer.get();
  }

  /**
   * Updates shift timing when the match clock crosses a configured teleop shift boundary.
   *
   * <p>This should be called periodically so the shift timer can restart at each boundary.
   */
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

  /**
   * Gets the remaining time in the current shift and updates {@link #currentShift}.
   *
   * @return seconds remaining in autonomous or in the active teleop shift
   */
  public static double getShiftTime() {
    double shiftTotalTime = 30.0;
    if (getTime() > 130) {
      shiftTotalTime = 10.0;
      currentShift = Shift.Transistion;
    } else if (getTime() > 30) {
      shiftTotalTime = 25.0;
      if (MathUtil.isNear(130.0, getTime(), 0.03)) {
        currentShift = Shift.Shift_1;
      }
      if (MathUtil.isNear(105.0, getTime(), 0.03)) {
        currentShift = Shift.Shift_2;
      }
      if (MathUtil.isNear(80.0, getTime(), 0.03)) {
        currentShift = Shift.Shift_3;
      }
      if (MathUtil.isNear(55.0, getTime(), 0.03)) {
        currentShift = Shift.Shift_4;
      }
    } else if (DriverStation.isAutonomous()) {
      shiftTotalTime = 20.0;
      currentShift = Shift.Autonomous;
    } else {
      currentShift = Shift.End_Game;
      shiftTotalTime = 30.0;
    }
    return (DriverStation.isAutonomous()
        ? autoLength - timer.get()
        : shiftTotalTime - shiftTimer.get());
  }

  /**
   * Determines whether the scoring hub is active for the given alliance matchup.
   *
   * @param startingAlliance alliance assigned to the robot at the start of the match cycle
   * @param currentAlliance alliance currently being evaluated
   * @return true when the hub is active for {@code currentAlliance}
   */
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

  /**
   * Checks whether a numeric value is within an inclusive range.
   *
   * @param value value to test
   * @param min inclusive minimum bound
   * @param max inclusive maximum bound
   * @return true when {@code value} is greater than or equal to {@code min} and less than or equal
   *     to {@code max}
   */
  private static boolean inRange(double value, double min, double max) {
    return value >= min && value <= max;
  }

  /**
   * Determines whether the hub should be considered active after accounting for projectile
   * time-of-flight near shift boundaries.
   *
   * @param currentAlliance alliance currently being evaluated
   * @param startingAlliance alliance assigned to the robot at the start of the match cycle
   * @param pose robot pose intended for time-of-flight estimation
   * @return true when the hub should be treated as active for the shot timing
   */
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

  /**
   * Checks whether the match clock is inside the time-of-flight window after a shift starts.
   *
   * @param shiftStart match time, in seconds remaining, when the shift begins
   * @param ToF projectile time of flight in seconds
   * @return true when the current match time is between {@code shiftStart} and {@code shiftStart +
   *     ToF}
   */
  private static boolean nearShift(double shiftStart, double ToF) {
    return inRange(getTime(), shiftStart, shiftStart + ToF);
  }
}

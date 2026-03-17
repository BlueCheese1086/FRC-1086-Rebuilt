// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private static final boolean IS_PRACTICE = !DriverStation.isFMSAttached();
  private static final String LOG_DIRECTORY = "/U/logs";
  private static final long MIN_FREE_SPACE =
      IS_PRACTICE
          ? 100000000
          : // 100 MB
          1000000000; // 1 GB

  public Robot() {
    // // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        File file = new File(LOG_DIRECTORY);
        System.out.println(
            file.exists() && file.isDirectory() ? "Logging to USB Drive" : "Logging to RoboRio");
        Logger.addDataReceiver(
            new WPILOGWriter(
                file.exists() && file.isDirectory() ? LOG_DIRECTORY : "home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        // setupLog();
        break;

      case SIM:
        // Running a physics simulator, log to NT
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // // Start AdvantageKit logger
    // Logger.start();
    // SignalLogger.enableAutoLogging(false);
    // HoodConstants.Targeting.hoodAngle = new LoggedTunableNumber("Hood/Hood Angle", 54.0);
    // try {
    //   Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
    //   watchdogField.setAccessible(true);
    //   Watchdog watchdog = (Watchdog) watchdogField.get(this);
    //   watchdog.setTimeout(0.2);
    // } catch (Exception e) {
    //   DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    // }
    // CommandScheduler.getInstance().setPeriod(0.2);

    // DriverStation.silenceJoystickConnectionWarning(true);

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    RobotController.setBrownoutVoltage(5.5);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Vision Transforms/Left", VisionConstants.robotToLeftCam);
    Logger.recordOutput("Vision Transforms/Right", VisionConstants.robotToRightCam);
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void setupLog() {
    // Check if the log directory exists
    File directory = new File(LOG_DIRECTORY);
    if (!directory.exists()) {
      directory.mkdir();
    }

    // ensure that there is enough space on the roboRIO to log data
    if (directory.getFreeSpace() < MIN_FREE_SPACE) {
      System.out.println("ERROR: out of space!");
      File[] files = directory.listFiles();
      if (files == null) {
        System.out.println("ERROR: Cannot delete, Files are NULL!");
      } else {
        // Sorting the files by name will ensure that the oldest files are deleted first
        files = Arrays.stream(files).sorted().toArray(File[]::new);

        long bytesToDelete = MIN_FREE_SPACE - directory.getFreeSpace();

        for (File file : files) {
          if (file.getName().endsWith(".wpilog")) {
            try {
              bytesToDelete -= Files.size(file.toPath());
            } catch (IOException e) {
              System.out.println("Failed to get size of file " + file.getName());
              continue;
            }
            if (file.delete()) {
              System.out.println("Deleted " + file.getName() + " to free up space");
            } else {
              System.out.println("Failed to delete " + file.getName());
            }
            if (bytesToDelete <= 0) {
              break;
            }
          }
        }
      }
    }
  }
}

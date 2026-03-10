// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseMath;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.0;

  // Lock Radius
  private static final double LOCK_RADIUS_KP = 1.5; // normalized output per meter error
  private static final double LOCK_RADIUS_MAX_OUTPUT = 0.8; // percent of max linear speed
  private static final double ANGLE_MAX_VELOCITY_LOCK = 50.0;
  private static final double ANGLE_MAX_ACCELERATION_LOCK = 70.0;

  @SuppressWarnings("unused")
  private static final double ANGLE_MAX_VELOCITY = 8.0;

  @SuppressWarnings("unused")
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          Translation2d robotVector =
              new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
          if (robotVector.getNorm() <= 1e-2) {
            drive.stopWithX();
          }
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command recordData(Drive drive, Shooter shooter, Hood hood) {
    Timer time = new Timer();
    return Commands.runEnd(
            () -> {
              shooter.setVelocitySetpoint(
                  () ->
                      RadiansPerSecond.of(ShooterConstants.Tuning.velocitySetpoint.getAsDouble()));
              // hood.setPosition(() -> hoodAngle.getAsDouble());
            },
            () -> {
              time.stop();
              shooter.recordShot(new Pose3d(drive.getPose()), hood.getAngle(), time.get());
              shooter.stopAll();
              Logger.recordOutput("File Writing/ Hood Angle", hood.getAngle());
              Logger.recordOutput("File Writing/Time", time.get());
              Logger.recordOutput("File Writing/Shot finished?", true);
            },
            shooter)
        .beforeStarting(
            () -> {
              time.reset();
              time.start();
              Logger.recordOutput("File Writing/Shot finished?", false);
            });
  }

  private static Pose2d target = Pose2d.kZero;

  public static Command moveBack(Drive drive) {
    @SuppressWarnings("resource")
    PIDController movementController = new PIDController(4.0, 0.0, 0.0);
    return Commands.run(
            () -> {
              Pose2d pose = drive.getPose();
              Logger.recordOutput("Auto Align/Target", target);
              double speed =
                  movementController.calculate(
                      pose.getTranslation().getNorm(), target.getTranslation().getNorm());
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(speed, 0.0, 0.0), drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              Pose2d pose = drive.getPose();
              target =
                  pose.transformBy(
                      new Transform2d(-Units.inchesToMeters(5.0), 0.0, Rotation2d.kZero));
            });
  }

  public static Command joystickDriveAtAngle(Drive drive, Supplier<Rotation2d> rotationSupplier) {
    // Create PID controller
    PIDController angleController = new PIDController(ANGLE_KP, 0.0, ANGLE_KD);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset());
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  @SuppressWarnings("resource")
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    PIDController angleController = new PIDController(ANGLE_KP, 0.0, ANGLE_KD);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset());
  }

  /**
   * Field relative drive where robot yaw is automatically pointed to the direction of travel.
   *
   * <p>This is useful for intaking: it overrides the turn joystick and keeps the intake pointed in
   * the direction the driver is commanding translation.
   */
  public static Command joystickDriveSyom(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Reuse the existing angle-hold command but source the target angle from the
    // translation stick. Add a deadband to avoid instability from stick drift.
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // If the driver isn't commanding translation, hold current heading.
          if (linearVelocity.getNorm() < 1e-2) { // increased deadband for stability
            return drive.getRotation();
          }

          // The translation direction is field-relative. Face that direction.
          return linearVelocity.getAngle();
        });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }
  /**
   * Field relative drive command that removes radial motion to a target (locks radius), while using
   * PID to continuously face the target.
   */
  public static Command joystickDriveLockRadiusToTarget(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> targetSupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller for heading
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY_LOCK, ANGLE_MAX_ACCELERATION_LOCK));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    final double[] lockedRadius = new double[] {0.0};

    return Commands.run(
            () -> {
              Pose2d robotPose = drive.getPose();
              Pose2d targetPose = targetSupplier.get();
              Rotation2d angle = rotationSupplier.get();

              // Get raw field-relative velocity from joysticks
              Translation2d rawVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Compute radial unit vector from target to robot
              Translation2d radial = robotPose.getTranslation().minus(targetPose.getTranslation());
              double radius = radial.getNorm();

              Translation2d tangentialVelocity = Translation2d.kZero;
              if (radius > 0.0001) {
                Translation2d radialUnit = radial.div(radius);
                double radialComponent =
                    rawVelocity.getX() * radialUnit.getX() + rawVelocity.getY() * radialUnit.getY();
                // Remove radial component to lock distance
                tangentialVelocity = rawVelocity.minus(radialUnit.times(radialComponent));

                // Add small radial correction to hold radius against drift
                double radiusError = radius - lockedRadius[0];
                double correction =
                    MathUtil.clamp(
                        -radiusError * LOCK_RADIUS_KP,
                        -LOCK_RADIUS_MAX_OUTPUT,
                        LOCK_RADIUS_MAX_OUTPUT);
                tangentialVelocity = tangentialVelocity.plus(radialUnit.times(correction));
              }

              // Calculate desired heading to face target
              Rotation2d targetHeading = PoseMath.getOrientationToTarget(robotPose, targetPose);
              // Rotation2d finalWithOffset =
              // new Rotation2d(targetHeading.getRadians() + Units.degreesToRadians(15.0));
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), angle.getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      tangentialVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      tangentialVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              boolean isFlipped = AllianceFlipUtil.shouldFlip();
              Logger.recordOutput(
                  "Lock Radius/Speeds",
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              lockedRadius[0] =
                  drive
                      .getPose()
                      .getTranslation()
                      .getDistance(targetSupplier.get().getTranslation());
            });
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}

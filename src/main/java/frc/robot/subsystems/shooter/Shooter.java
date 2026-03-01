// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shooter.FeederIO.FeederIO;
import frc.robot.subsystems.shooter.FeederIO.FeederIOInputsAutoLogged;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Hub;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterInputsAutoLogged[] inputs;
  private ShooterIO[] io;
  private FeederIO feederIO;
  private FeederIOInputsAutoLogged feederIOInputsAutoLogged;
  private final File file;

  public Shooter(FeederIO feederIO, ShooterIO... io) {
    this.io = io;
    this.feederIO = feederIO;
    this.feederIOInputsAutoLogged = new FeederIOInputsAutoLogged();
    inputs = new ShooterInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ShooterInputsAutoLogged();
    }
    file = new File(ShooterConstants.Targeting.FileName);

    try (FileWriter writer = new FileWriter(file, true)) {
      if (file.length() == 0) {
        writer.write("Distance, Shooter, Angle, TOF\n");
      } else {
        clearFile(file);
        writer.write("Distance, Shooter, Angle, TOF\n");
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private void clearFile(File file) {
    try (FileWriter writer = new FileWriter(file, false)) {
      writer.write("");
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void setVelocitySetpoint(Supplier<AngularVelocity> radPerSec) {
    for (int i = 0; i < io.length; i++) {
      io[i].setVelocity(radPerSec.get());
    }
  }

  public Command runFeed(double volts) {
    return Commands.run(() -> feederIO.setFeedVoltage(volts), this).finallyDo(() -> stopFeeder());
  }

  public Command runShooter(Supplier<AngularVelocity> velocity) {
    return Commands.run(() -> setVelocitySetpoint(velocity), this).until(() -> atSetpoint());
  }

  public Command stopShoot() {
    return Commands.runOnce(() -> stopShooter());
  }

  public Command stopFeed() {
    return Commands.runOnce(() -> stopFeed());
  }

  public Command stopEverything() {
    return Commands.runOnce(() -> stopAll());
  }

  public Command setVoltage(double volts) {
    return Commands.run(
        () -> {
          for (int i = 0; i < io.length; i++) {
            io[i].setVoltage(volts);
          }
        }).finallyDo(()-> stopShoot());
  }

  public void stopAll() {
    for (int i = 0; i < io.length; i++) {
      io[i].setVoltage(0.0);
      io[i].setVelocity(RadiansPerSecond.of(0.0));
    }
    feederIO.setFeedVoltage(0.0);
  }

  public void stopShooter() {
    for (int i = 0; i < io.length; i++) {
      io[i].setVoltage(0.0);
      io[i].setVelocity(RadiansPerSecond.of(0.0));
    }
  }

  public void stopFeeder() {
    feederIO.setFeedVoltage(0.0);
  }

  /**
   * @return returns the setpoint of the MIDDLE SHOOTER, if that shooter is at the
   *         setpoint or not
   */
  public boolean atSetpoint() {
    return inputs[0].atSetpoint && inputs[1].atSetpoint && inputs[2].atSetpoint;
  }

  public void recordShot(Pose3d drivePose, Angle hoodAngle, double tof) {
    double distanceToHub = Math.abs(
        drivePose
            .plus(ShooterConstants.ShooterTransforms.centerShooter)
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(Hub.topCenterPoint)));
    Logger.recordOutput("File Writing/ Distance to Hub", distanceToHub);
    Logger.recordOutput(
        "File Writing/ Shooter RPM",
        Units.radiansPerSecondToRotationsPerMinute(inputs[1].velocity));

    try (FileWriter writer = new FileWriter(file, true)) {
      writer.append(
          distanceToHub
              + " ,"
              + inputs[1].velocity
              + " ,"
              + hoodAngle.in(Degrees)
              + " , "
              + tof
              + "\n");
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public Pose3d[] getShooterPoses(Pose2d robotPose) {
    Pose3d robot3d = new Pose3d(robotPose);
    return new Pose3d[] {
        robot3d.transformBy(ShooterConstants.ShooterTransforms.leftShooter),
        robot3d.transformBy(ShooterConstants.ShooterTransforms.centerShooter),
        robot3d.transformBy(ShooterConstants.ShooterTransforms.rightShooter)
    };
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Shooter/Flywheel" + (i + 1), inputs[i]);
    }
    feederIO.updateInputs(feederIOInputsAutoLogged);
    Logger.processInputs("Shooter/Feeder", feederIOInputsAutoLogged);
  }

  public Command sysid(double timeout, int i, String string) {
    return Commands.sequence(
        this.getShooterSysIdQuasistatic(Direction.kForward, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        this.getShooterSysIdQuasistatic(Direction.kReverse, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        this.getShooterSysIdDynamic(Direction.kForward, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        this.getShooterSysIdDynamic(Direction.kReverse, i, string).withTimeout(timeout),
        Commands.waitUntil(() -> inputs[i].velocity <= 30.0),
        Commands.runOnce(() -> io[i].setVoltage(0.0)));
  }

  public Command getShooterSysIdQuasistatic(Direction direction, int index, String name) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null),
        new SysIdRoutine.Mechanism(
            volts -> io[index].setVoltage(volts.in(Volts)),
            log -> {
              log.motor(name)
                  .voltage(Volts.of(inputs[index].appliedVoltage))
                  .angularPosition(Rotations.of(inputs[index].positionRadPerSec))
                  .angularVelocity(RotationsPerSecond.of(inputs[index].velocity));
            },
            this))
        .quasistatic(direction);
  }

  public Command getShooterSysIdDynamic(Direction direction, int index, String name) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null),
        new SysIdRoutine.Mechanism(
            volts -> io[index].setVoltage(volts.in(Volts)),
            log -> {
              log.motor(name)
                  .voltage(Volts.of(inputs[index].appliedVoltage))
                  .angularPosition(Rotations.of(inputs[index].positionRadPerSec))
                  .angularVelocity(RotationsPerSecond.of(inputs[index].velocity));
            },
            this))
        .dynamic(direction);
  }
}

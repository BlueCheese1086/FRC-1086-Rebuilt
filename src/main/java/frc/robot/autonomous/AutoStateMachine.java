package frc.robot.autonomous;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import choreo.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTransforms;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator.LaunchingParameters;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoStateMachine {
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Hood hood;

  public final Field2d autoPreviewField = new Field2d();

  public AutoStateMachine(Drive drive, Shooter shooter, Indexer indexer, Intake intake, Hood hood) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.hood = hood;
  }

  private double addPathToPreview(String trajName, List<Pose2d> previewPoses) {
    var traj = Choreo.loadTrajectory(trajName);
    if (traj.isPresent()) {
      Pose2d[] poses = traj.get().getPoses();

      for (Pose2d pose : poses) {
        previewPoses.add(AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.apply(pose) : pose);
      }

      return traj.get().getTotalTime();
    }
    return 0.0;
  }

  public Command buildAutoSequence(
      String startPos,
      String preloadShootPos,
      String intakePos,
      String nzEntry,
      String nzExit,
      String finalShootPos,
      String climbPos,
      double shootTime,
      double intakeTime) {

    Command autoCommands = Commands.sequence();
    String currentLocation = startPos;
    List<Pose2d> previewPoses = new ArrayList<>();

    double estimatedTime = 0.0;

    // PRELOAD SHOOT
    if (startPos.endsWith("r")) {
      autoCommands =
          autoCommands.andThen(startShoot(shootTime), Commands.waitSeconds(0.5), stopShoot());
    } else if (!preloadShootPos.equals("none")) {
      String path = currentLocation + "_" + preloadShootPos;
      estimatedTime += addPathToPreview(path, previewPoses);

      autoCommands =
          autoCommands.andThen(
              // Using the marker-aware path runner
              Commands.deadline(AutoRoutines.runPath(path, true)),
              startShoot(shootTime),
              Commands.waitSeconds(shootTime));
      estimatedTime += shootTime;
      currentLocation = preloadShootPos;
    }

    boolean isFirstPath = currentLocation.equals(startPos);

    // INTAKE SEQUENCE
    if (intakePos.endsWith("i")) {
      String path = currentLocation + "_" + intakePos;
      estimatedTime += addPathToPreview(path, previewPoses);
      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(path, isFirstPath), intake()),
              Commands.waitSeconds(intakeTime));
      estimatedTime += intakeTime;
      currentLocation = intakePos;
    } else {
      String entryPath = currentLocation + "_" + nzEntry;
      String intakePath = nzEntry + "_" + intakePos;
      estimatedTime += addPathToPreview(entryPath, previewPoses);
      estimatedTime += addPathToPreview(intakePath, previewPoses);

      autoCommands =
          autoCommands.andThen(
              AutoRoutines.runPath(entryPath, isFirstPath),
              Commands.deadline(AutoRoutines.runPath(intakePath, false), intake()),
              Commands.waitSeconds(intakeTime));
      estimatedTime += intakeTime;
      currentLocation = intakePos;
    }

    // FINAL SHOOT SEQUENCE
    if (intakePos.endsWith("n")) {
      String exitPath = currentLocation + "_" + nzExit;
      String safePath = nzExit + "_" + nzExit + "s";
      String shootPath = nzExit + "s_" + finalShootPos;

      estimatedTime += addPathToPreview(exitPath, previewPoses);
      estimatedTime += addPathToPreview(safePath, previewPoses);
      estimatedTime += addPathToPreview(shootPath, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(exitPath, false)),
              AutoRoutines.runPath(safePath, false),
              AutoRoutines.runPath(shootPath, false),
              startShoot(shootTime));
      estimatedTime += shootTime;
      currentLocation = finalShootPos;
    } else {
      String shootPath = currentLocation + "_" + finalShootPos;
      estimatedTime += addPathToPreview(shootPath, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(shootPath, false)),
              startShoot(shootTime),
              stopShoot());
      estimatedTime += shootTime;
      currentLocation = finalShootPos;
    }

    if (!climbPos.equals("none")) {
      String climbPath = currentLocation + "_" + climbPos;
      estimatedTime += addPathToPreview(climbPath, previewPoses);
      autoCommands =
          autoCommands.andThen(
              AutoRoutines.runPath(climbPath, false)
              // TODO: do climb stuff later
              );
    }

    autoPreviewField.getObject("traj").setPoses(previewPoses);

    SmartDashboard.putString("Auto time", String.format("%.2f", estimatedTime) + "s");

    return autoCommands.finallyDo(
        () -> {
          drive.stop();
          stopShoot();
          // TODO: add more stops if needed
        });
  }

  public Command startFeeder() { // just spins up flywheel, TODO: make better
    return Commands.runOnce(() -> shooter.runFeed(12).execute());
  }

  public Command startShoot(double shootTime) {
    return Commands.parallel(
            Commands.waitSeconds(2.0)
                .andThen(
                    Commands.parallel(
                        shooter.runFeed(ShooterConstants.FeederSetpoints.run.in(Volts))
                        /*.finallyDo(shooter.runFeederVoltage(0.0)::execute)*/ ,
                        indexer.setVoltage(IndexerConstants.Setpoints.feed),
                        Commands.repeatingSequence( // TODO: uhh probaly not gonna agitate
                            intake.setPosition(IntakeConstants.Setpoints.agitate),
                            intake.setPosition(IntakeConstants.Setpoints.deployed)))),
            runFlywheel())
        .withTimeout(shootTime);
  }

  public Command runFlywheel() {
    return Commands.run(
        () -> {
          LaunchingParameters parms =
              LauncherCalculator.getInstance()
                  .getParameters(
                      () ->
                          (new Pose3d(drive.getPose())
                              .transformBy(ShooterTransforms.centerShooter)
                              .toPose2d()),
                      drive::getChassisSpeeds,
                      drive::getRotation);
          //   shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(350.0));
          shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(parms.flywheelSpeed()));
          hood.setPosition(() -> parms.hoodAngle());

          Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
          Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
          Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
          Logger.recordOutput("Shoot Parms/Distance", parms.distance());
        });
  }

  public Command stopShoot() {
    return Commands.run(shooter::stopAll)
        .withTimeout(0.01)
        .andThen(indexer.setVoltage(Volts.zero()))
        .andThen(hood.setAngle(HoodConstants.Setpoints.passAngle));
  }

  public Command deployIntake() {
    return intake.setPosition(IntakeConstants.Setpoints.deployed);
  }

  public Command intake() {
    return Commands.parallel(
        intake.setVoltage(IntakeConstants.Setpoints.run),
        indexer.setVoltage(IndexerConstants.Setpoints.feed));
  }

  public Command retractIntake() {
    return intake.setPosition(IntakeConstants.Setpoints.stowed);
  }
}

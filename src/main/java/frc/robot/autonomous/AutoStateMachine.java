package frc.robot.autonomous;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import choreo.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;

public class AutoStateMachine {
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;

  public final Field2d autoPreviewField = new Field2d();

  public AutoStateMachine(Drive drive, Shooter shooter, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
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

    if (startPos.endsWith("r")) {
      autoCommands = autoCommands.andThen(startShoot(), Commands.waitSeconds(0.5), stopShoot());
    } else if (!preloadShootPos.equals("none")) {
      String path = currentLocation + "_" + preloadShootPos;
      estimatedTime += addPathToPreview(path, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(path, true), stopIntake()),
              startShoot(),
              Commands.waitSeconds(shootTime));
      estimatedTime += shootTime;
      currentLocation = preloadShootPos;
    }

    boolean isFirstPath = currentLocation.equals(startPos);

    if (intakePos.endsWith("i")) {
      String path = currentLocation + "_" + intakePos;
      estimatedTime += addPathToPreview(path, previewPoses);
      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(path, isFirstPath), startIntake()), //TODO: make it so that the intake does not drop when outpost intake
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
              Commands.deadline(AutoRoutines.runPath(intakePath, false), startIntake()),
              Commands.waitSeconds(intakeTime));
      estimatedTime += intakeTime;
      currentLocation = intakePos;
    }

    if (intakePos.endsWith("n")) {
      String exitPath = currentLocation + "_" + nzExit;
      String safePath = nzExit + "_" + nzExit + "s";
      String shootPath = nzExit + "s_" + finalShootPos;

      estimatedTime += addPathToPreview(exitPath, previewPoses);
      estimatedTime += addPathToPreview(safePath, previewPoses);
      estimatedTime += addPathToPreview(shootPath, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(exitPath, false), stopIntake()),
              AutoRoutines.runPath(safePath, false),
              AutoRoutines.runPath(shootPath, false),
              startShoot(),
              Commands.waitSeconds(shootTime));
      estimatedTime += shootTime;
      currentLocation = finalShootPos;
    } else {
      String shootPath = currentLocation + "_" + finalShootPos;
      estimatedTime += addPathToPreview(shootPath, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.deadline(AutoRoutines.runPath(shootPath, false), stopIntake()),
              startShoot(),
              Commands.waitSeconds(shootTime));
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
          stopIntake();
          // TODO: add more stops if needed
        });
  }

  public Command startShoot() {
    return Commands.runOnce(
        () ->
            shooter.setVelocitySetpoint(
                RadiansPerSecond.of(ShooterConstants.Tuning.velocitySetpoint.getAsDouble())),
        shooter);
  }

  public Command stopShoot() {
    return Commands.runOnce(shooter::stopAll);
  }

  public Command startIntake() {
    return Commands.parallel(
        intake.setPosition(IntakeConstants.Setpoints.deployed),
        indexer.setVoltage(IndexerConstants.Setpoints.feed));
  }

  public Command stopIntake() {
    return Commands.parallel(
        intake.setPosition(IntakeConstants.Setpoints.stowed), indexer.setVoltage(Volts.zero()));
  }
}

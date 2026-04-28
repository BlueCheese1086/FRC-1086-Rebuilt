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

  private Command pathCommand(String pathName, boolean resetPose, boolean previewOnly) {
    return previewOnly ? Commands.none() : AutoRoutines.runPath(pathName, resetPose);
  }

  public Command buildAutoSequence(
      String startPos,
      String preloadShootPos,
      int swipeCount,
      String nzEntry,
      String nzExit,
      String finalShootPos,
      String climbPos,
      double shootTime,
      double unused,
      boolean previewOnly) {

    Command autoCommands = Commands.sequence();
    String currentLocation = startPos;
    List<Pose2d> previewPoses = new ArrayList<>();

    double estimatedTime = 0.0;

    // shoot preload
    if (!preloadShootPos.equals("none")) {
      String path = currentLocation + "_" + preloadShootPos;
      estimatedTime += addPathToPreview(path, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.parallel(
                  pathCommand(path, true, previewOnly), runFlywheel(), retractIntake()),
              startShoot(shootTime));

      estimatedTime += shootTime;
      currentLocation = preloadShootPos;
    }

    boolean isFirstPath = currentLocation.equals(startPos);

    boolean useSwipes = swipeCount > 0 && (nzEntry.equals("db") || nzEntry.equals("ob"));

    if (useSwipes) {
      String entryPath = currentLocation + "_" + nzEntry;
      estimatedTime += addPathToPreview(entryPath, previewPoses);
      autoCommands = autoCommands.andThen(pathCommand(entryPath, isFirstPath, previewOnly));
      currentLocation = nzEntry;
      isFirstPath = false;

      for (int swipeIndex = 1; swipeIndex <= swipeCount; swipeIndex++) {
        String swipeType = swipeIndex % 2 == 1 ? "farswipe" : "nearswipe";
        String swipePath = nzEntry + "_" + swipeType;
        estimatedTime += addPathToPreview(swipePath, previewPoses);

        autoCommands =
            autoCommands.andThen(
                pathCommand(swipePath, false, previewOnly).deadlineFor(deployIntake(), intake()));

        String shootPath = nzEntry + "_" + finalShootPos;
        estimatedTime += addPathToPreview(shootPath, previewPoses);
        autoCommands =
            autoCommands.andThen(
                Commands.parallel(
                    pathCommand(shootPath, false, previewOnly), runFlywheel(), retractIntake()),
                startShoot(shootTime));
        estimatedTime += shootTime;

        if (swipeIndex < swipeCount) {
          String returnPath = finalShootPos + "_" + nzEntry;
          estimatedTime += addPathToPreview(returnPath, previewPoses);
          autoCommands = autoCommands.andThen(pathCommand(returnPath, false, previewOnly));
          currentLocation = nzEntry;
        } else {
          currentLocation = finalShootPos;
        }
      }
    } else {
      String shootPath = currentLocation + "_" + finalShootPos;
      estimatedTime += addPathToPreview(shootPath, previewPoses);

      autoCommands =
          autoCommands.andThen(
              Commands.parallel(
                  pathCommand(shootPath, false, previewOnly), runFlywheel(), retractIntake()),
              startShoot(shootTime));
      estimatedTime += shootTime;
      currentLocation = finalShootPos;
    }

    if (!climbPos.equals("none")) {
      String climbPath = currentLocation + "_" + climbPos;
      estimatedTime += addPathToPreview(climbPath, previewPoses);
      autoCommands =
          autoCommands.andThen(
              pathCommand(climbPath, false, previewOnly)
              // TODO: climb commands
              );
    }

    autoPreviewField.getObject("traj").setPoses(previewPoses);

    SmartDashboard.putString("Auto time", String.format("%.2f", estimatedTime) + "s");

    return autoCommands.finallyDo(
        () -> {
          drive.stop();
          stopShoot();
          retractIntake();
        });
  }

  public Command startShoot(double shootTime) {
    return getShootCommand().withTimeout(shootTime);
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
              shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(parms.flywheelSpeed()));
              hood.setPosition(() -> parms.hoodAngle());
            })
        .until(shooter::atSetpoint);
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
    return intake.setVoltage(IntakeConstants.Setpoints.run);
  }

  public Command retractIntake() {
    return intake.setPosition(IntakeConstants.Setpoints.stowed);
  }

  private Command getShootCommand() {
    return Commands.parallel(
        Commands.run(
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
                  shooter.setVelocitySetpoint(() -> RadiansPerSecond.of(parms.flywheelSpeed()));
                  hood.setPosition(() -> parms.hoodAngle());
                  Logger.recordOutput("Shoot Parms/ Hood Angle", parms.hoodAngle());
                  Logger.recordOutput("Shoot Parms/ Drive Angle", parms.driveAngle());
                  Logger.recordOutput("Shoot Parms/ Flywheel Speed", parms.flywheelSpeed());
                  Logger.recordOutput("Shoot Parms/Distance", parms.distance());
                },
                shooter)
            .finallyDo(shooter::stopShooter),
        Commands.waitSeconds(1.0)
            .andThen(
                Commands.parallel(
                    shooter
                        .runFeed(ShooterConstants.FeederSetpoints.run.in(Volts))
                        .finallyDo(shooter::stopFeeder),
                    indexer.setVoltage(IndexerConstants.Setpoints.feed),
                    intake.setVoltage(IntakeConstants.Setpoints.run))),
        Commands.repeatingSequence(
            intake.setPosition(IntakeConstants.Setpoints.agitate),
            Commands.waitSeconds(0.2),
            intake.setPosition(IntakeConstants.Setpoints.deployed),
            Commands.waitSeconds(0.2)));
  }
}

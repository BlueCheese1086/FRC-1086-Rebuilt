package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTransforms;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator;
import frc.robot.subsystems.shooter.shooterUtil.LauncherCalculator.LaunchingParameters;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
  private final Drive drive;
  private final AutoFactory factory;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  private final Hood hood;

  public AutoRoutines(Drive drive, Shooter shooter, Intake intake, Indexer indexer, Hood hood) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;
    this.hood = hood;
    factory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive.choreoDriveController(),
            true,
            drive,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
            });
  }

  public AutoFactory getFactory() {
    return factory;
  }

  public Command getRBAuto() {
    final var routine = factory.newRoutine("Choreo Auto");
    final var first = routine.trajectory("RB1");
    final var second = routine.trajectory("RB2");
    final var third = routine.trajectory("RB3");
    final var fourth = routine.trajectory("RB4");

    routine
        .active()
        .whileTrue(
            Commands.sequence(
                DriveCommands.autoAlign(drive, () -> first.getInitialPose().orElse(new Pose2d()))
                    .until(
                        () ->
                            DriveCommands.isNear(
                                first.getInitialPose().orElse(new Pose2d()), drive.getPose())),
                first.resetOdometry(),
                first.cmd(),
                second.resetOdometry(),
                Commands.deadline(second.cmd(), intake.setVoltage(IntakeConstants.Setpoints.run)),
                third.resetOdometry(),
                Commands.deadline(third.cmd(), intake.setVoltage(IntakeConstants.Setpoints.run)),
                Commands.deadline(Commands.waitSeconds(6.0), getShootCommand()),
                intake.setPosition(IntakeConstants.Setpoints.deployed),
                fourth.resetOdometry(),
                Commands.deadline(fourth.cmd(), intake.setVoltage(IntakeConstants.Setpoints.run)),
                Commands.deadline(Commands.waitSeconds(6.0), getShootCommand())));

    routine.observe(first.done()).onTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));

    return routine.cmd();
  }

  public Command getLBAuto() {
    final var routine = factory.newRoutine("Choreo Auto");
    final var first = routine.trajectory("RB1").mirrorY();
    final var second = routine.trajectory("RB2").mirrorY();
    final var third = routine.trajectory("RB3").mirrorY();
    final var fourth = routine.trajectory("RB4").mirrorY();

    routine
        .active()
        .whileTrue(
            Commands.sequence(
                DriveCommands.autoAlign(drive, () -> first.getInitialPose().orElse(new Pose2d()))
                    .until(
                        () ->
                            DriveCommands.isNear(
                                first.getInitialPose().orElse(new Pose2d()), drive.getPose())),
                first.resetOdometry(),
                first.cmd(),
                second.resetOdometry(),
                Commands.deadline(second.cmd(), intake.setVoltage(IntakeConstants.Setpoints.run)),
                third.resetOdometry(),
                Commands.deadline(third.cmd(), intake.setVoltage(IntakeConstants.Setpoints.run)),
                Commands.deadline(Commands.waitSeconds(6.0), getShootCommand()),
                intake.setPosition(IntakeConstants.Setpoints.deployed),
                fourth.resetOdometry(),
                Commands.deadline(fourth.cmd(), intake.setVoltage(IntakeConstants.Setpoints.run)),
                Commands.deadline(Commands.waitSeconds(6.0), getShootCommand())));

    routine.observe(first.done()).onTrue(intake.setPosition(IntakeConstants.Setpoints.deployed));

    return routine.cmd();
  }

  public Command LMtoH() {
    final var routine = factory.newRoutine("LM to H");
    final var score = routine.trajectory("LMtoH");
    routine.active().whileTrue(Commands.sequence(score.resetOdometry(), score.cmd()));
    return routine.cmd();
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
        Commands.waitSeconds(0.35)
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
            Commands.waitSeconds(0.2)),
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 0.0,
            () -> 0.0,
            () ->
                LauncherCalculator.getInstance()
                    .getParameters(drive::getPose, drive::getChassisSpeeds, drive::getRotation)
                    .driveAngle()));
  }
}

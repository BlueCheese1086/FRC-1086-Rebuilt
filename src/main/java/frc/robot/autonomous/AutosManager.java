// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Set;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class AutosManager extends SubsystemBase {
  private final SendableChooser<String> startPos = new SendableChooser<>();
  private final SendableChooser<String> preloadShootPos = new SendableChooser<>();
  private final SendableChooser<String> intakePos = new SendableChooser<>();
  private final SendableChooser<String> nzEntry = new SendableChooser<>();
  private final SendableChooser<String> nzExit = new SendableChooser<>();
  private final SendableChooser<String> finalShootPos = new SendableChooser<>();
  private final SendableChooser<String> climbPos = new SendableChooser<>();

  private final Field2d autoPreviewField = new Field2d();

  private final Drive drive;
  private final BooleanSupplier isRed;
  private final AutoStateMachine machine;

  public record Auto(String name, Command command, Pose2d initPose) {}

  public AutosManager(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake, BooleanSupplier isRed) {
    this.drive = drive;
    this.isRed = isRed;
    this.machine = new AutoStateMachine(drive, shooter, indexer, intake, isRed);

    initUI();
  }

  public void initUI() {
    startPos.setDefaultOption("Hub Start", "hs");
    startPos.addOption("Depot Trench Start", "dts");
    startPos.addOption("Depot Bump Start", "dbs");
    startPos.addOption("Outpost Trench Start", "ots");
    startPos.addOption("Outpost Bump Start", "obs");
    startPos.addOption("Hub Start Reverse", "hsr");
    startPos.addOption("Depot Trench Start Reverse", "dtsr");
    startPos.addOption("Depot Bump Start Reverse", "dbsr");
    startPos.addOption("Outpost Trench Start Reverse", "otsr");
    startPos.addOption("Outpost Bump Start Reverse", "obsr");

    preloadShootPos.setDefaultOption("Center Shot", "cs");
    preloadShootPos.addOption("Depot Far Shot", "dfs");
    preloadShootPos.addOption("Depot Near Shot", "dns");
    preloadShootPos.addOption("Outpost Far Shot", "ofs");
    preloadShootPos.addOption("Outpost Near Shot", "ons");
    preloadShootPos.addOption("None", "none");

    intakePos.setDefaultOption("Depot", "di");
    intakePos.addOption("Outpost", "oi");
    intakePos.addOption("Depot Close Neutral", "dcn");
    intakePos.addOption("Outpost Close Neutral", "ocn");
    intakePos.addOption("Depot Far Safe Neutral", "dfsn");
    intakePos.addOption("Depot Near Safe Neutral", "dnsn");
    intakePos.addOption("Outpost Far Safe Neutral", "ofsn");
    intakePos.addOption("Outpost Near Safe Neutral", "onsn");
    intakePos.addOption("Center Risky Neutral", "crn");
    intakePos.addOption("Depot Far Risky Neutral", "dfrn");
    intakePos.addOption("Depot Near Risky Neutral", "dnrn");
    intakePos.addOption("Outpost Far Risky Neutral", "ofrn");
    intakePos.addOption("Outpost Near Risky Neutral", "onrn");

    nzEntry.setDefaultOption("Depot Trench", "dt");
    nzEntry.addOption("Depot Bump", "db");
    nzEntry.addOption("Outpost Trench", "ot");
    nzEntry.addOption("Outpost Bump", "ob");

    nzExit.setDefaultOption("Depot Trench", "dt");
    nzExit.addOption("Depot Bump", "db");
    nzExit.addOption("Outpost Trench", "ot");
    nzExit.addOption("Outpost Bump", "ob");

    finalShootPos.setDefaultOption("Center Shot", "cs");
    finalShootPos.addOption("Depot Far Shot", "dfs");
    finalShootPos.addOption("Depot Near Shot", "dns");
    finalShootPos.addOption("Outpost Far Shot", "ofs");
    finalShootPos.addOption("Outpost Near Shot", "ons");

    climbPos.setDefaultOption("None", "none");
    climbPos.addOption("Depot Climb", "dc");
    climbPos.addOption("Outpost Climb", "oc");

    SmartDashboard.putData("Auto/Start Pos", startPos);
    SmartDashboard.putData(
        "Auto/Preload Shoot Pos (if preloaded and not a reverse starting position)",
        preloadShootPos);
    SmartDashboard.putData(
        "Auto/Intake Source (after shooting preload or starting not preloaded)", intakePos);
    SmartDashboard.putData(
        "Auto/NZ Entry (if starting or going to neutral zone after shooting preload)", nzEntry);
    SmartDashboard.putData("Auto/NZ Exit (if entered neutral zone)", nzExit);
    SmartDashboard.putData("Auto/Final Shoot Pos", finalShootPos);
    SmartDashboard.putData("Auto/Climb Pos", climbPos);

    SmartDashboard.putData("Auto Path", autoPreviewField);

    SmartDashboard.putBoolean("Auto/Rebuild Preview", false);
  }

  public Command getSelectedAuto() {
    return Commands.defer(
        () ->
            machine.buildAutoSequence(
                startPos.getSelected(),
                preloadShootPos.getSelected(),
                intakePos.getSelected(),
                nzEntry.getSelected(),
                nzExit.getSelected(),
                finalShootPos.getSelected(),
                climbPos.getSelected(),
                1.0,
                1.0),
        Set.of(drive));
  }

  public void updatePreview() {
    if (SmartDashboard.getBoolean("Auto/Rebuild Preview", false)) {
      SmartDashboard.putBoolean("Auto/Rebuild Preview", false);

      machine.buildAutoSequence(
          startPos.getSelected(),
          preloadShootPos.getSelected(),
          intakePos.getSelected(),
          nzEntry.getSelected(),
          nzExit.getSelected(),
          finalShootPos.getSelected(),
          climbPos.getSelected(),
          1.0,
          1.0);

      var pathPoses = machine.autoPreviewField.getObject("traj").getPoses();
      autoPreviewField.getObject("traj").setPoses(pathPoses);
    }

    autoPreviewField.setRobotPose(drive.getPose());
  }

  @Override
  public void periodic() {
    updatePreview();
  }
}

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class AutoBuilder {
  private final Drive drive;

  private final SendableChooser<String> startPos = new SendableChooser<>();
  private final SendableChooser<String> preloadShootPos = new SendableChooser<>();
  private final SendableChooser<String> intakePos = new SendableChooser<>();
  private final SendableChooser<String> nzEntry = new SendableChooser<>();
  private final SendableChooser<String> nzExit = new SendableChooser<>();
  private final SendableChooser<String> finalShootPos = new SendableChooser<>();
  private final SendableChooser<String> climbPos = new SendableChooser<>();
  private final Field2d autoTraj = new Field2d();

  private Command commands;

  public AutoBuilder(Drive drive) { // TODO: take in superstructure instead of drive - no need - Martin
    this.drive = drive;

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

    SmartDashboard.putData("Auto Path", autoTraj);

    SmartDashboard.putBoolean("Auto/Rebuild Preview", false);
  }

  public Command build() {
    return Commands.defer(
        () -> {
          rebuildPreview();
          return commands;
        },
        Set.of(drive));
  }

  // TODO: Implement these commands when superstructure is ready (or once I figure out what's wrong)
  private Command shootCommand() {
    return print(""); // superstructure.setState(Superstructure.State.shoot);
  }

  private Command intakeCommand() {
    return print(""); /*superstructure
        .setState(Superstructure.State.intake)
        .withTimeout(1.0)
        .andThen(superstructure.setState(Superstructure.State.holding));*/
  }

  private Command climbCommand() {
    return print(""); /*superstructure
        .setState(Superstructure.State.climb)
        .withTimeout(1.0)
        .andThen(superstructure.setState(Superstructure.State.climbscore));*/
  }

  public static List<Pose2d> getPathPoses(String trajectory) {
    var traj = Choreo.loadTrajectory(trajectory);
    if (traj.isPresent()) {
      Pose2d[] poses = traj.get().getPoses();
      for (int i = 0; i < poses.length; i++) {
        poses[i] = AllianceFlipUtil.apply(poses[i]);
      }
      return Arrays.asList(poses);
    }
    return new ArrayList<>();
  }

  public void updateField() {
    autoTraj.setRobotPose(drive.getPose());

    if (SmartDashboard.getBoolean("Auto/Rebuild Preview", false)) {
      SmartDashboard.putBoolean("Auto/Rebuild Preview", false);
      rebuildPreview();
    }
  }

  private void rebuildPreview() {
    String _startPos = startPos.getSelected();
    String _preloadShootPos = preloadShootPos.getSelected();
    String _intakePos = intakePos.getSelected();
    String _nzEntry = nzEntry.getSelected();
    String _nzExit = nzExit.getSelected();
    String _finalShootPos = finalShootPos.getSelected();
    String _climbPos = climbPos.getSelected();

    List<Pose2d> p = new ArrayList<Pose2d>();
    List<Command> c = new ArrayList<Command>();

    if (_startPos.endsWith("r")) {
      c.add(shootCommand());
    }
    if (_preloadShootPos.equals("none")) {
      if (_intakePos.endsWith("i")) {
        c.add(AutoRoutines.runPath(_startPos + "_" + _intakePos, true));
        p.addAll(getPathPoses(_startPos + "_" + _intakePos));
      } else {
        c.add(AutoRoutines.runPath(_startPos + "_" + _nzEntry, true));
        p.addAll(getPathPoses(_startPos + "_" + _nzEntry));
        c.add(AutoRoutines.runPath(_nzEntry + "_" + _intakePos, false));
        p.addAll(getPathPoses(_nzEntry + "_" + _intakePos));
      }
    } else {
      c.add(AutoRoutines.runPath(_startPos + "_" + _preloadShootPos, true));
      p.addAll(getPathPoses(_startPos + "_" + _preloadShootPos));
      c.add(shootCommand());
      if (_intakePos.endsWith("i")) {
        c.add(AutoRoutines.runPath(_preloadShootPos + "_" + _intakePos, false));
        p.addAll(getPathPoses(_preloadShootPos + "_" + _intakePos));
      } else {
        c.add(AutoRoutines.runPath(_preloadShootPos + "_" + _nzEntry, false));
        p.addAll(getPathPoses(_preloadShootPos + "_" + _nzEntry));
        c.add(AutoRoutines.runPath(_nzEntry + "_" + _intakePos, false));
        p.addAll(getPathPoses(_nzEntry + "_" + _intakePos));
      }
    }
    c.add(intakeCommand());
    if (_intakePos.endsWith("n")) {
      c.add(AutoRoutines.runPath(_intakePos + "_" + _nzExit, false));
      p.addAll(getPathPoses(_intakePos + "_" + _nzExit));
      c.add(AutoRoutines.runPath(_nzExit + "_" + _nzExit + "s", false));
      p.addAll(getPathPoses(_nzExit + "_" + _nzExit + "s"));
      c.add(AutoRoutines.runPath(_nzExit + "s_" + _finalShootPos, false));
      p.addAll(getPathPoses(_nzExit + "s_" + _finalShootPos));
    } else {
      c.add(AutoRoutines.runPath(_intakePos + "_" + _finalShootPos, false));
      p.addAll(getPathPoses(_intakePos + "_" + _finalShootPos));
    }
    c.add(shootCommand());
    if (!_climbPos.equals("none")) {
      c.add(AutoRoutines.runPath(_finalShootPos + "_" + _climbPos, false));
      p.addAll(getPathPoses(_finalShootPos + "_" + _climbPos));
      c.add(climbCommand());
    }

    autoTraj.getObject("traj").setPoses(p);

    commands = Commands.sequence(c.toArray(new Command[0]));
  }
}

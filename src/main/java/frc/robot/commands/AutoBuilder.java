package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import java.util.Set;

public class AutoBuilder {
  private final Superstructure superstructure;

  private final SendableChooser<String> startPos = new SendableChooser<>();
  private final SendableChooser<String> preloadShootPos = new SendableChooser<>();
  private final SendableChooser<String> intakePos = new SendableChooser<>();
  private final SendableChooser<String> nzEntry = new SendableChooser<>();
  private final SendableChooser<String> nzExit = new SendableChooser<>();
  private final SendableChooser<String> finalShootPos = new SendableChooser<>();
  private final SendableChooser<String> climbPos = new SendableChooser<>();

  public AutoBuilder(Superstructure superstructure) {
    this.superstructure = superstructure;

    startPos.setDefaultOption("Hub Start", "hs");
    startPos.addOption("Depot Trench Start", "dts");
    startPos.addOption("Depot Bump Start", "dbs");
    startPos.addOption("Outpost Trench Start", "ots");
    startPos.addOption("Outpost Bump Start", "obs");

    preloadShootPos.setDefaultOption("Center Shot", "cs");
    preloadShootPos.addOption("Depot Far Shot", "dfs");
    preloadShootPos.addOption("Depot Near Shot", "dns");
    preloadShootPos.addOption("Outpost Far Shot", "ofs");
    preloadShootPos.addOption("Outpost Near Shot", "ons");
    preloadShootPos.addOption("None", "none");

    intakePos.setDefaultOption("Depot", "di");
    intakePos.addOption("Outpost", "oi");
    intakePos.setDefaultOption("Depot Close Neutral", "dcn");
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
    SmartDashboard.putData("Auto/Preload Shoot Pos (if preloaded)", preloadShootPos);
    SmartDashboard.putData(
        "Auto/Intake Source (after shooting preload or starting not preloaded)", intakePos);
    SmartDashboard.putData(
        "Auto/NZ Entry (if at Hub Start or going to neutral zone after shooting preload)", nzEntry);
    SmartDashboard.putData("Auto/NZ Exit (if entered neutral zone)", nzExit);
    SmartDashboard.putData("Auto/Final Shoot Pos", finalShootPos);
    SmartDashboard.putData("Auto/Climb Pos", climbPos);
  }

  public Command build() {
    return Commands.defer(
        () -> {
          String _startPos = startPos.getSelected();
          String _preloadShootPos = preloadShootPos.getSelected();
          String _intakePos = intakePos.getSelected();
          String _nzEntry = nzEntry.getSelected();
          String _nzExit = nzExit.getSelected();
          String _finalShootPos = finalShootPos.getSelected();
          String _climbPos = climbPos.getSelected();

          String _selectedEntry =
              _startPos.equals("hs")
                  ? _nzEntry
                  : _startPos.substring(
                      0,
                      _startPos.length()
                          - 1); // if starting at hub, use selected nz entry, else use closest entry

          return Commands.sequence(
              (_preloadShootPos.equals("none"))
                  ? // if not preloaded
                  ((_intakePos.endsWith("i"))
                      ? // if not going to neutral zone
                      AutoRoutines.runPath(_startPos + "_" + _intakePos, true)
                      : // go directly to intake position
                      AutoRoutines.runPath(
                              _startPos + "_" + _selectedEntry,
                              true) // else go to closest entry, then intake
                          .andThen(AutoRoutines.runPath(_selectedEntry + "_" + _intakePos, false)))
                  : // if preloaded
                  AutoRoutines.runPath(
                          _startPos + "_" + _preloadShootPos,
                          true) // go to shoot position, shoot, then go to intake
                      // position
                      .andThen(shootCommand())
                      .andThen(
                          (_intakePos.endsWith("i"))
                              ? // if not going to neutral zone
                              AutoRoutines.runPath(_preloadShootPos + "_" + _intakePos, false)
                              : // go directly to intake position
                              AutoRoutines.runPath(
                                      _preloadShootPos + "_" + _nzEntry,
                                      false) // else go to selected entry, then intake
                                  .andThen(
                                      AutoRoutines.runPath(_nzEntry + "_" + _intakePos, false))),
              intakeCommand(), // intake
              (_intakePos.endsWith("n"))
                  ? // if in neutral zone
                  AutoRoutines.runPath(_intakePos + "_" + _nzExit, false) // go to the exit
                      .andThen(
                          AutoRoutines.runPath(
                              _nzExit + "_" + _nzExit + "s",
                              false)) // short go to closest start position
                      .andThen(AutoRoutines.runPath(_nzExit + "s_" + _finalShootPos, false))
                  : // use existing path to go to final shoot position
                  AutoRoutines.runPath(
                      _intakePos + "_" + _finalShootPos,
                      false), // else, just go to final shoot position
              shootCommand(), // shoot
              (_climbPos.equals("none"))
                  ? // if climbing, go climb but if not do nothing
                  Commands.none()
                  : AutoRoutines.runPath(_finalShootPos + "_" + _climbPos, false)
                      .andThen(climbCommand()));
        },
        Set.of(superstructure));
  }
  
  private Command shootCommand() {
    return superstructure.setState(Superstructure.State.score);
  }

  private Command intakeCommand() {
    return superstructure.setState(Superstructure.State.intake);
  }

  private Command climbCommand() {
    return superstructure.setState(Superstructure.State.climb)
      .withTimeout(1.0)
      .andThen(superstructure.setState(Superstructure.State.climbscore));
  }
}

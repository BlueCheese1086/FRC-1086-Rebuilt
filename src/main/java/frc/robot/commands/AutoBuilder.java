package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoBuilder {
  private final SendableChooser<String> startPos = new SendableChooser<>();
  private final SendableChooser<String> preloadShootPos = new SendableChooser<>();
  private final SendableChooser<String> intakePos = new SendableChooser<>();
  private final SendableChooser<String> nzEntry = new SendableChooser<>();
  private final SendableChooser<String> nzExit = new SendableChooser<>();
  private final SendableChooser<String> finalShootPos = new SendableChooser<>();
  private final SendableChooser<String> climbPos = new SendableChooser<>();

  public AutoBuilder() {
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
    SmartDashboard.putData("Auto/Intake Source (after shooting preload or starting not preloaded)", intakePos);
    SmartDashboard.putData("Auto/NZ Entry (if at Hub Start or going to neutral zone after shooting preload)", nzEntry);
    SmartDashboard.putData("Auto/NZ Exit (if entered neutral zone)", nzExit);
    SmartDashboard.putData("Auto/Final Shoot Pos", finalShootPos);
    SmartDashboard.putData("Auto/Climb Pos", climbPos);
  }

  public Command build() {
    String _startPos = startPos.getSelected();
    String _preloadShootPos = preloadShootPos.getSelected();
    String _intakePos = intakePos.getSelected();
    String _nzEntry = nzEntry.getSelected();
    String _nzExit = nzExit.getSelected();
    String _finalShootPos = finalShootPos.getSelected();
    String _climbPos = climbPos.getSelected();

    return Commands.sequence(
      (_preloadShootPos.equals("none")) ?
        ((_startPos.equals("hs")) ? AutoRoutines.runPath(_startPos + "_" + _nzEntry) : Commands.none())
          .andThen(AutoRoutines.runPath(_startPos + "_" + _intakePos)) :
        AutoRoutines.runPath(_startPos + "_" + _preloadShootPos)
          .andThen(dummyShoot())
          .andThen(AutoRoutines.runPath(_preloadShootPos + "_" + _intakePos)),
      
      dummyIntake(),

      (_intakePos.endsWith("n")) ?
        AutoRoutines.runPath(_intakePos + "_" + _nzExit)
          .andThen(AutoRoutines.runPath(_nzExit + "_" + _finalShootPos)) :
        AutoRoutines.runPath(_intakePos + "_" + _finalShootPos),
      dummyShoot(),

      (_climbPos.equals("none")) ? 
        Commands.none() : 
        AutoRoutines.runPath(_climbPos + "_climb")
          .andThen(dummyClimb())
    );
  }

  // TODO: put actual commands here
  private Command dummyShoot() {
    return print("shoot");
  }

  private Command dummyIntake() {
    return print("intake");
  }

  private Command dummyClimb() {
    return print("climb");
  }
}

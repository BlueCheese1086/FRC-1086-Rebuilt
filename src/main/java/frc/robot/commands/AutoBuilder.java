package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoBuilder {
  private final SendableChooser<String> startPos = new SendableChooser<>();
  private final SendableChooser<Boolean> shouldShoot = new SendableChooser<>();
  private final SendableChooser<String> preloadShootPos = new SendableChooser<>();
  private final SendableChooser<String> intakeType = new SendableChooser<>();
  private final SendableChooser<String> nzEntry = new SendableChooser<>();
  private final SendableChooser<String> nzTarget = new SendableChooser<>();
  private final SendableChooser<String> nzExit = new SendableChooser<>();
  private final SendableChooser<String> finalShootPos = new SendableChooser<>();
  private final SendableChooser<String> climbPos = new SendableChooser<>();

  public AutoBuilder() {
    startPos.setDefaultOption("Hub Start", "hs");
    startPos.addOption("Depot Trench Start", "dts");
    startPos.addOption("Depot Bump Start", "dbs");
    startPos.addOption("Outpost Trench Start", "ots");
    startPos.addOption("Outpost Bump Start", "obs");

    shouldShoot.setDefaultOption("Yes", true);
    shouldShoot.addOption("No", false);

    preloadShootPos.setDefaultOption("Center Shot", "cs");
    preloadShootPos.addOption("Depot Far Shot", "dfs");
    preloadShootPos.addOption("Depot Near Shot", "dns");
    preloadShootPos.addOption("Outpost Far Shot", "ofs");
    preloadShootPos.addOption("Outpost Near Shot", "ons");

    intakeType.setDefaultOption("Depot", "di");
    intakeType.addOption("Outpost", "oi");
    intakeType.addOption("Neutral Zone (None)", "none");

    nzEntry.setDefaultOption("Depot Trench", "dt");
    nzEntry.addOption("Depot Bump", "db");
    nzEntry.addOption("Outpost Trench", "ot");
    nzEntry.addOption("Outpost Bump", "ob");

    nzTarget.setDefaultOption("Depot Close Neutral", "dcn");
    nzTarget.addOption("Outpost Close Neutral", "ocn");
    nzTarget.addOption("Depot Mid Neutral", "dmn");
    nzTarget.addOption("Outpost Mid Neutral", "omn");
    nzTarget.addOption("Depot Far Neutral", "dfn");
    nzTarget.addOption("Depot Near Neutral", "dnn");
    nzTarget.addOption("Outpost Far Neutral", "ofn");
    nzTarget.addOption("Outpost Near Neutral", "onn");

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

    SmartDashboard.putData("Auto/1. Start Pos", startPos);
    SmartDashboard.putData("Auto/2. Preloaded", shouldShoot);
    SmartDashboard.putData("Auto/3. Preload Shoot Pos", preloadShootPos);
    SmartDashboard.putData("Auto/4. Intake Source", intakeType);
    SmartDashboard.putData("Auto/5a. NZ Entry", nzEntry);
    SmartDashboard.putData("Auto/5b. NZ Target", nzTarget);
    SmartDashboard.putData("Auto/5c. NZ Exit", nzExit);
    SmartDashboard.putData("Auto/6. Final Shoot Pos", finalShootPos);
    SmartDashboard.putData("Auto/7. Climb Pos", climbPos);
  }

  public Command build() {
    String start = startPos.getSelected();
    boolean startShoot = shouldShoot.getSelected();
    String pShoot = preloadShootPos.getSelected();
    String intake = intakeType.getSelected();
    String fShoot = finalShootPos.getSelected();
    String climb = climbPos.getSelected();

        return AutoRoutines.runPath(
            startShoot ? //if it should shoot,
                (start + "-" + pShoot) : //go to shooting position
                (start + "-" + getIntakePoint()) //else go to intake point
        ).andThen(
            startShoot ? //shoot and go to intake if preloaded
                dummyShoot().andThen(AutoRoutines.runPath(pShoot + "-" + getIntakePoint())) : 
                Commands.none(),
            intake.equals("none") ? //if no intake was selected,
                AutoRoutines.runPath(nzEntry.getSelected() + "-" + nzTarget.getSelected()).andThen( //go to neutral zone target
                    dummyIntake(), //intake
                    AutoRoutines.runPath(nzTarget.getSelected() + "-" + nzExit.getSelected()), //go to exit
                    AutoRoutines.runPath(nzExit.getSelected() + "-" + fShoot) //go to shoot
                ) : 
                dummyIntake().andThen(AutoRoutines.runPath(intake + "-" + fShoot)), //else go to shoot
            dummyShoot(), //shoot
            !climb.equals("none") ? //if climb selected,
                AutoRoutines.runPath(fShoot + "-" + climb).andThen(dummyClimb()) : //go to climb positon and climb
                Commands.none()
        );
    }

    private String getIntakePoint() { //get intake point based on intake type (either neutral zone or depot/outpost)
        return intakeType.getSelected().equals("none") ? nzEntry.getSelected() : intakeType.getSelected();
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

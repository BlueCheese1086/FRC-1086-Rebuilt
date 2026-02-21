package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import choreo.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.autonomous.old.AutoRoutines;

public class AutoStateMachine {
    private final Superstructure superstructure;
    private final Drive drive;
    private final BooleanSupplier isRed;

    public final Field2d autoPreviewField = new Field2d();

    public AutoStateMachine(Superstructure superstructure, Drive drive, BooleanSupplier isRed) {
        this.superstructure = superstructure;
        this.drive = drive;
        this.isRed = isRed;
    }

    private void addPathToPreview(String trajName, List<Pose2d> previewPoses){
        var traj = Choreo.loadTrajectory(trajName);
        if (traj.isPresent()) {
            Pose2d[] poses = traj.get().getPoses();
            boolean flip = isRed.getAsBoolean();
            
            for (Pose2d pose : poses) {
                previewPoses.add(flip ? AllianceFlipUtil.apply(pose) : pose);
            }
        }
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

        if (startPos.endsWith("r")) {
            autoCommands = autoCommands.andThen(
                superstructure.setState(Superstructure.State.shoot),
                Commands.waitSeconds(0.5),
                superstructure.setState(Superstructure.State.idle)
            );
        } else if (!preloadShootPos.equals("none")) {
            String path = currentLocation + "_" + preloadShootPos;
            addPathToPreview(path, previewPoses);
            
            autoCommands = autoCommands.andThen(
                Commands.deadline(
                    AutoRoutines.runPath(path, true), // Reset odometry on the first path
                    superstructure.setState(Superstructure.State.holding) // Prep while moving
                ),
                superstructure.setState(Superstructure.State.shoot),
                Commands.waitSeconds(shootTime) // Wait for piece to leave
            );
            currentLocation = preloadShootPos;
        }

        boolean isFirstPath = currentLocation.equals(startPos);

        if (intakePos.endsWith("i")) {
            String path = currentLocation + "_" + intakePos;
            addPathToPreview(path, previewPoses);
            autoCommands = autoCommands.andThen(
                Commands.deadline(
                    AutoRoutines.runPath(path, isFirstPath),
                    superstructure.setState(Superstructure.State.intake) // Intake drops while driving
                ),
                // Path is done but keep intake down until sensor detects a piece
                Commands.waitSeconds(intakeTime) 
            );
            currentLocation = intakePos;
        } else {
            String entryPath = currentLocation + "_" + nzEntry;
            String intakePath = nzEntry + "_" + intakePos;
            addPathToPreview(entryPath, previewPoses);
            addPathToPreview(intakePath, previewPoses);

            autoCommands = autoCommands.andThen(
                AutoRoutines.runPath(entryPath, isFirstPath),
                Commands.deadline(
                    AutoRoutines.runPath(intakePath, false),
                    superstructure.setState(Superstructure.State.intake) // Drop intake going into zone
                ),
                Commands.waitSeconds(intakeTime)
            );
            currentLocation = intakePos;
        }

        if (intakePos.endsWith("n")) {
            String exitPath = currentLocation + "_" + nzExit;
            String safePath = nzExit + "_" + nzExit + "s";
            String shootPath = nzExit + "s_" + finalShootPos;

            addPathToPreview(exitPath, previewPoses);
            addPathToPreview(safePath, previewPoses);
            addPathToPreview(shootPath, previewPoses);

            autoCommands = autoCommands.andThen(
                Commands.deadline(
                    AutoRoutines.runPath(exitPath, false),
                    superstructure.setState(Superstructure.State.holding) // Stow intake while leaving
                ),
                AutoRoutines.runPath(safePath, false),
                AutoRoutines.runPath(shootPath, false),
                superstructure.setState(Superstructure.State.shoot),
                Commands.waitSeconds(0.5)
            );
            currentLocation = finalShootPos;
        } else {
            String shootPath = currentLocation + "_" + finalShootPos;
            addPathToPreview(shootPath, previewPoses);
            
            autoCommands = autoCommands.andThen(
                Commands.deadline(
                    AutoRoutines.runPath(shootPath, false),
                    superstructure.setState(Superstructure.State.holding) // Stow intake
                ),
                superstructure.setState(Superstructure.State.shoot),
                Commands.waitSeconds(0.5)
            );
            currentLocation = finalShootPos;
        }

        if (!climbPos.equals("none")) {
            String climbPath = currentLocation + "_" + climbPos;
            addPathToPreview(climbPath, previewPoses);
            autoCommands = autoCommands.andThen(
                AutoRoutines.runPath(climbPath, false),
                superstructure.setState(Superstructure.State.climb),
                Commands.waitSeconds(1.0),
                superstructure.setState(Superstructure.State.climbscore)
            );
        }

        autoPreviewField.getObject("traj").setPoses(previewPoses);

        // Ensure everything stops and stows when auto ends
        return autoCommands.finallyDo(() -> {
            drive.stop();
            superstructure.setState(Superstructure.State.idle).schedule();
        });
    }
}
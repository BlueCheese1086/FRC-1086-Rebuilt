package frc.robot.autonomous;

import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public record AutoSubsystems(
    Drive drive,
    Shooter shooter,
    Intake intake,
    Climb climb
) {}

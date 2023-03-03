package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreBalanceAuto extends SequentialCommandGroup {
    public ScoreBalanceAuto(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new ZeroPitchRollCommand(driveTrain),
            new AutoScore(driveTrain, extendo),
            new TimedIntakeCommand(intake, false),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 2.4),
            new AutoLevelPIDCommand(driveTrain)
        );
    }
}

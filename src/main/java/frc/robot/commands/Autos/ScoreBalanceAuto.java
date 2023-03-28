package frc.robot.commands.Autos;

import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ScoreBalanceAuto extends SequentialCommandGroup {
    public ScoreBalanceAuto(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new TimedIntakeCommand(intake, true, 0.5, -0.8),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new ZeroPitchRollCommand(driveTrain),
            new AutoScore(extendo, "cube high"),
            new AutoScoreExtend(extendo, "cube high"),
            new TimedIntakeCommand(intake, false, 1.5, 0.8),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 2.4),
            new AutoLevelPIDCommand(driveTrain)
        );
    }
}

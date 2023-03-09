package frc.robot.commands.Autos;

import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreBalanceAuto extends SequentialCommandGroup {

    public ScoreBalanceAuto(DrivetrainSubsystem driveTrain, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
                new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
                new ZeroGyroscopeCommand(driveTrain, 180),
                new ZeroPitchRollCommand(driveTrain),
                new AutoScore(driveTrain, extendo),
                new TimedIntakeCommand(driveTrain, intake),
                new DriveAndHomeCommand(driveTrain, extendo, intake),
                new AutoLevelPIDCommand(driveTrain));
    }
}

package frc.robot.commands.Autos;

import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ScoreEngageAndPickup extends SequentialCommandGroup {
    public ScoreEngageAndPickup(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new ZeroPitchRollCommand(driveTrain),
            new AutoScore(extendo, "cube"),
            new TimedIntakeCommand(intake, false),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 3.5),
            new RotateMoveIntakeDrive(driveTrain, vision, extendo, intake),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 1.2),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 4.7),
            new AutoLevelPIDCommand(driveTrain)
        );
    }
}

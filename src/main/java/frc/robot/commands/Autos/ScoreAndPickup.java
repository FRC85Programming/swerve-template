package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.SideDependentStrafe;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScoreAndPickup extends SequentialCommandGroup {

    public ScoreAndPickup(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new zeroWheels(driveTrain),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new HomeExtendCommand(extendo),
            new AutoScore(extendo, "cone middle"),
            new AutoScoreExtend(extendo, "cone middle"),
            new SideDependentStrafe(driveTrain, 0.5),
            new TimedIntakeCommand(intake, true, 1.5, 0.8),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 3.3),
            new RotateAndIntakePosition(driveTrain, vision, extendo, intake),
            new ScoreLineup(driveTrain, vision, robotContainer, true),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 2),
            new HomeExtendCommand(extendo)
        );
    }
    
}

 
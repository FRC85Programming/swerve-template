package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.AutoLevelCommand;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScoreAndPickup extends SequentialCommandGroup {

    public ScoreAndPickup(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new zeroWheels(driveTrain),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new AutoScore(extendo, "cone middle"),
            new AutoScoreExtend(extendo, "cone middle"),
            new TimedIntakeCommand(intake, false, 1.5),
            new StrafeAndHome(driveTrain, vision, extendo, intake, 0.5),
            new DriveDistance(driveTrain, vision, 0, -1.5, 0, 4.5, 0, false),
            new RotateAndIntakePosition(driveTrain, vision, extendo, intake),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 2),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, -5),
            new AutoScore(extendo, "cube middle"),
            new AutoScoreExtend(extendo, "cube middle"),
            new TimedIntakeCommand(intake, null, 1.5),
            new HomeExtendCommand(extendo)          
        );
    }
    
}

 
package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScoreAndPickup extends SequentialCommandGroup {

    public ScoreAndPickup(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new zeroWheels(driveTrain),
            new ZeroGyroscopeCommand(driveTrain, 0),
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new AutoScore(extendo, "cube"),
            new TimedIntakeCommand(intake, false),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, SmartDashboard.getNumber("Drive and Home Distance", 0)),
            new RotateAndIntakePosition(driveTrain, vision, extendo, intake),
            //new ScoreLineup(driveTrain, vision),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 3),
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new DriveDistance(driveTrain, vision, 0, 0, 1, 0, 100, false),
            new DriveDistance(driveTrain, vision, 0, 2, 0, -7.5),
            new DriveDistance(driveTrain, vision, 0, 0, -5, 0, -5,false),
            new AutoScore(extendo, "cube"), 
            new TimedIntakeCommand(intake, false),   
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0)
            
        );
    }
}

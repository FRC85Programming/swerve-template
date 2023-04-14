package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScorePickupAndBalance extends SequentialCommandGroup{

    public ScorePickupAndBalance(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new ZeroGyroscopeCommand(driveTrain, 180),
            new HomeExtendCommand(extendo),
            new zeroWheels(driveTrain),
            new TimedIntakeCommand(intake, false, .5, 0.4),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 4.3, -1.3, false),
            new WaitCommand(0.2),
            new RotateAndIntakePosition(driveTrain, vision, extendo, intake, Alliance.Blue, "cone"),
            new DriveDistance(driveTrain, vision, -0.5, 0, 0, 0.2, 0, false),
            //new zeroWheels(driveTrain),
            new WaitCommand(0.2),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 1.2, "cone"),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, -3.4, -1.3, false),
            new AutoLevelPIDCommand(driveTrain)
        );
    }
    
}
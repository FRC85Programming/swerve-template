package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScorePickupAndBalance extends SequentialCommandGroup {

    public ScorePickupAndBalance(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(  
            new HomeExtendCommand(extendo),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new zeroWheels(driveTrain),
            new TimedIntakeCommand(intake, false, 0.8, 0.8),
            new DriveDistance(driveTrain, vision, 0, -1, 0, 0.5, 0, null),
            new DriveDistance(driveTrain, vision, 0, 0, 2, 0, 170, null),
            new DriveAndConeIntake(driveTrain, vision, extendo, intake, 5),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 2.5),
            new AutoLevelPIDCommand(driveTrain)
        );
    }
}
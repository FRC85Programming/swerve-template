package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScoreAndPickupNoVision extends SequentialCommandGroup{

    public ScoreAndPickupNoVision(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake, Alliance side) {
        double strafeSpeed;
        if (side == Alliance.Blue){
            strafeSpeed = -0.5;
        } else {
            strafeSpeed = 0.5;
        }
        
        addCommands(
            new ZeroGyroscopeCommand(driveTrain, 180),
            new HomeExtendCommand(extendo),
            new AutoScore(extendo, "cone middle"),
            new AutoScoreExtend(extendo, "cone middle"),
            new TimedIntakeCommand(intake, true, 1.5, 0.8),
            new DriveDistance(driveTrain, vision, strafeSpeed, 0, 0, 0.3, 0, false),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 3.5),
            new RotateAndIntakePosition(driveTrain, vision, extendo, intake, side),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 1.3),
            new HomeExtendCommand(extendo)
        );
    }
    
}

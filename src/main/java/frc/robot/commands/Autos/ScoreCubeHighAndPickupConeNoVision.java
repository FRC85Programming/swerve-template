package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroWheelsSideways;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScoreCubeHighAndPickupConeNoVision extends SequentialCommandGroup{

    public ScoreCubeHighAndPickupConeNoVision(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake, Alliance allianceSide, String autoSide) {
        double strafeSpeed;

        if (autoSide.toLowerCase().equals("Bumpside")) {

            if (allianceSide == Alliance.Blue){
                strafeSpeed = -0.5;
            } else {
                strafeSpeed = 0.5;
            }

        } else if(autoSide.toLowerCase().equals("SmoothSide")) {
            if (allianceSide == Alliance.Blue){
                strafeSpeed = 0.5;
            } else {
                strafeSpeed = -0.5;
            }
        } else {
            strafeSpeed = 0;
        }
        
        addCommands(
            new ZeroGyroscopeCommand(driveTrain, 180),
            new HomeExtendCommand(extendo),
            new zeroWheels(driveTrain),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new AutoScore(extendo, "cube high"),
            new AutoScoreExtend(extendo, "cube high"),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new TimedIntakeCommand(intake, false, .5, 0.4),
            new DriveDistance(driveTrain, vision, strafeSpeed, 0, 0, 0.13, 0, false, 0.75),
            new zeroWheels(driveTrain), 
            new WaitCommand(0.1),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 3.5, -2.5, true, 0.75),
            new RotateAndIntakePosition(driveTrain, vision, extendo, intake, allianceSide, "cone"),
            new zeroWheels(driveTrain),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 1.3, "cone"),
            new HomeExtendCommand(extendo)
            /*new DriveDistance(driveTrain, vision, -strafeSpeed, 0, 0, 0.3, 0, false),
            new DriveDistance(driveTrain, vision, 0, 0, 2, 0, 170, null),
            new DriveDistance(driveTrain, vision, 0, 2, 0, 3, 0, null),
            new TimedIntakeCommand(intake, false, 0.5, -1)*/
        );
    }
    
}
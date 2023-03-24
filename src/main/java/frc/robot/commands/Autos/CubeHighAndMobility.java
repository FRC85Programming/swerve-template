package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class CubeHighAndMobility extends SequentialCommandGroup {

    public CubeHighAndMobility(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(  
            new HomeExtendCommand(extendo),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new TimedIntakeCommand(intake, true, 0.5),
            new AutoScore(extendo, "cube high"),
            new AutoScoreExtend(extendo, "cube high"),
            new TimedIntakeCommand(intake, false, 1.5),
            new HomeExtendCommand(extendo),
            new DriveDistance(driveTrain, vision, 0, -1, 0.0, 3.8, 0, false),
            new AutoIntakeCube(extendo, intake)
        );
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ManualCS extends SequentialCommandGroup {
    public ManualCS(DrivetrainSubsystem driveTrain, VisionTracking vision, IntakeSubsystem intake, RobotContainer robotContainer) {
        addCommands(  
            new zeroWheels(driveTrain),
            new DriveDistance(driveTrain, vision, 0, 5.0, 0.0, 3),
            new DriveDistance(driveTrain, vision, 5, 0, 0, 2),
            new WaitCommand(.3),
            new DriveDistance(driveTrain, vision, 0, -5, 0, 2),
            new AutoLevelCommand(driveTrain)


        );
    }
}
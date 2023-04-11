package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.AutoLevelCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;

public class ManualCS extends SequentialCommandGroup {
    public ManualCS(DrivetrainSubsystem driveTrain, VisionTracking vision, IntakeSubsystem intake, RobotContainer robotContainer) {
        addCommands(  
            new zeroWheels(driveTrain),
            new DriveDistance(driveTrain, vision, 0, 5.0, 0.0, 3, 0, false),
            new DriveDistance(driveTrain, vision, 5, 0, 0, 2, 0, false),
            new WaitCommand(.3),
            new DriveDistance(driveTrain, vision, 0, -5, 0, 2, 0, false),
            new AutoLevelCommand(driveTrain)
        );
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ManualMobility extends SequentialCommandGroup {
    public ManualMobility(DrivetrainSubsystem driveTrain, IntakeSubsystem intake, RobotContainer robotContainer) {
        addCommands(  
            //new zeroWheels(driveTrain),
            new DriveDistance(driveTrain, 0, 0.5, 0.0, 4.95, 0)
        );
    }
}

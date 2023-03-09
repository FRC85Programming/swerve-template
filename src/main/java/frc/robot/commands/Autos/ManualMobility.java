package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.subsystems.*;

public class ManualMobility extends SequentialCommandGroup {
    public ManualMobility(DrivetrainSubsystem driveTrain, RobotContainer robotContainer) {
        addCommands(
                // new zeroWheels(driveTrain),
                new DriveDistance(driveTrain, 0, 0.5, 0.0, 4.95));
    }
}

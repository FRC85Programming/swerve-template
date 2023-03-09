package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.AutoLevelCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;

public class ManualCS extends SequentialCommandGroup {
    public ManualCS(DrivetrainSubsystem driveTrain, RobotContainer robotContainer) {
        addCommands(
                new zeroWheels(driveTrain),
                new DriveDistance(driveTrain, 0, 5.0, 0.0, 3),
                new DriveDistance(driveTrain, 5, 0, 0, 2),
                new WaitCommand(.3),
                new DriveDistance(driveTrain, 0, -5, 0, 2),
                new AutoLevelCommand(driveTrain));
    }
}
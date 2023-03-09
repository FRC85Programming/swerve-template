package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.subsystems.*;

public class ManualOnePlace extends SequentialCommandGroup {

    public ManualOnePlace(DrivetrainSubsystem driveTrain, RobotContainer robotContainer, ExtendoSubsystem extendo,
            IntakeSubsystem intake) {
        addCommands(
                new ZeroGyroscopeCommand(driveTrain, 180),
                new AutoScore(driveTrain, extendo),
                new TimedIntakeCommand(driveTrain, intake),
                new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
                new DriveDistance(driveTrain, 0, -1.0, 0.0, 4.95));
    }
}

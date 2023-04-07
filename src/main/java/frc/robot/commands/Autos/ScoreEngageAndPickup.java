package frc.robot.commands.Autos;

import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ScoreEngageAndPickup extends SequentialCommandGroup {
    public ScoreEngageAndPickup(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new zeroWheels(driveTrain),
            new ZeroGyroscopeCommand(driveTrain, 0)
        );
    }
}

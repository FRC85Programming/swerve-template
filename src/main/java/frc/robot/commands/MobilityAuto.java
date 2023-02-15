package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class MobilityAuto extends SequentialCommandGroup {
    public MobilityAuto(DrivetrainSubsystem DRIVE) {
        addCommands(
            new DriveDistance(DRIVE, 5, 0, 5)
        );
    }
}
package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ConeMidAndMobility extends SequentialCommandGroup {

    public ConeMidAndMobility(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(  
            new HomeExtendCommand(extendo),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new TimedIntakeCommand(intake, true, 0.5),
            new AutoScore(extendo, "cone middle"),
            new AutoScoreExtend(extendo, "cone middle"),
            new TimedIntakeCommand(intake, false, 1.5),
            new HomeExtendCommand(extendo)
        );
    }
}

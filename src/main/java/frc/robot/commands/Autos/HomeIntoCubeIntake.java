package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class HomeIntoCubeIntake extends SequentialCommandGroup {

    public HomeIntoCubeIntake(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        addCommands(
            new HomeExtendCommand(m_ExtendoSubsystem),
            new AutoIntakeCube(m_ExtendoSubsystem, m_IntakeSubsystem)
        );
    }

}

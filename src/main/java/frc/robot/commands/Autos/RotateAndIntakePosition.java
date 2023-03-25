package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class RotateAndIntakePosition extends ParallelCommandGroup {

    public RotateAndIntakePosition(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem){

        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, vision, 0, 0, 2.5, 0, 180, false),
            new AutoIntakeCube(m_ExtendoSubsystem, m_IntakeSubsystem)
        );
    }

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveAndHomeCommand extends ParallelCommandGroup {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;

    public DriveAndHomeCommand(DrivetrainSubsystem m_DrivetrainSubsystem, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem){
        this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
        this.m_ExtendoSubsystem = m_ExtendoSubsystem;
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        addCommands(
            new ExtendCommand(m_ExtendoSubsystem, () -> 0, () -> 0, () -> 0),
            new DriveDistance(m_DrivetrainSubsystem,0, -0.85, 0, 2.6)
        );
    }

}

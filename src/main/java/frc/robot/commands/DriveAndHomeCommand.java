package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class DriveAndHomeCommand extends ParallelCommandGroup {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;

    public DriveAndHomeCommand(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
        this.m_ExtendoSubsystem = m_ExtendoSubsystem;
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, vision, 0, -5, 0, target, 0, false),
            new ExtendCommand(m_ExtendoSubsystem, () -> 0, () -> 0, () -> 0)
        );
    }

}

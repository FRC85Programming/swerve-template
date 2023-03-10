package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveAndIntake extends ParallelCommandGroup {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;

    public DriveAndIntake(DrivetrainSubsystem m_DrivetrainSubsystem, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
        this.m_ExtendoSubsystem = m_ExtendoSubsystem;
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        SmartDashboard.putNumber("Intake Start", 1);
        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, 0, 1, 0, target, 0),
            new TimedIntakeCommand(m_IntakeSubsystem, true)
        );
    }

}
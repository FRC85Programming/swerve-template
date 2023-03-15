package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class DriveAndIntake extends ParallelCommandGroup {
    public DriveAndIntake(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        SmartDashboard.putNumber("Intake Start", 1);
        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, vision, 0, 1.5, 0, target, 0, false),
            new TimedIntakeCommand(m_IntakeSubsystem, true)
        );
    }

}
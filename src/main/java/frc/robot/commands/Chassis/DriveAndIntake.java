package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class DriveAndIntake extends ParallelCommandGroup {
    public DriveAndIntake(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target, String gamePiece){
        SmartDashboard.putNumber("Intake Start", 1);
        boolean intakeMode = gamePiece.toLowerCase().equals("cube");
        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, vision, 0, 1, 0, target, 0, false),
            new TimedIntakeCommand(m_IntakeSubsystem, intakeMode, 2, -0.8)
        );
    }

}
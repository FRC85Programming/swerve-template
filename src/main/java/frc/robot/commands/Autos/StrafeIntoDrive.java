package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class StrafeIntoDrive extends SequentialCommandGroup {

    public StrafeIntoDrive(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, vision, -1, -1, 0, 0.85, 0, false),
            new DriveDistance(m_DrivetrainSubsystem, vision, 0, -1.5, 0, 1.5, 0, false)
        );
    }

}
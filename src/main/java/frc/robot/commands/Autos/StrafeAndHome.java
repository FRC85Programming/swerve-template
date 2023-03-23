package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class StrafeAndHome extends ParallelCommandGroup {

    public StrafeAndHome(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        addCommands(
            new StrafeIntoDrive(m_DrivetrainSubsystem, vision, m_ExtendoSubsystem, m_IntakeSubsystem, target),
            new HomeIntoCubeIntake(m_DrivetrainSubsystem, vision, m_ExtendoSubsystem, m_IntakeSubsystem, target)
        );
    }

}

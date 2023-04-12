package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class DriveAndConeIntake extends ParallelCommandGroup {

    public DriveAndConeIntake(DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, double target){
        addCommands(
            new DriveDistance(m_DrivetrainSubsystem, vision, 0, 1, 0, target, 0, null),
            new ConeIntakeIntoConePickup(m_DrivetrainSubsystem, vision, m_IntakeSubsystem, m_ExtendoSubsystem)
        );
    }

}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BrakeWheelsCommand extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public BrakeWheelsCommand(DrivetrainSubsystem driveTrain)
    {
        this.m_drivetrainSubsystem = driveTrain;
        
        }

    @Override
    public void execute()
    {
        m_drivetrainSubsystem.setLock(true);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_drivetrainSubsystem.setLock(false);
    }
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MobilityAuto extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public MobilityAuto(DrivetrainSubsystem driveTrain) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
    }


    @Override
    public void execute() {
        
    }

    @Override
    public void end (boolean interrupted)  {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
    }
}
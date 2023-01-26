package frc.robot.commands;

import javax.swing.text.AbstractDocument.BranchElement;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;

public class TrackReflective extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;
    public TrackReflective(DrivetrainSubsystem driveTrain, VisionTracking visionTracking){

        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
   }


    @Override
    public void execute() {
     
    }
    @Override
    public void end(boolean interrupted)  {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}

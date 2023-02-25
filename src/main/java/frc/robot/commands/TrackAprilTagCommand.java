package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class TrackAprilTagCommand extends CommandBase
{
    double wheelAngle = 0.0;
    boolean wheelReset = true;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;
    public TrackAprilTagCommand(DrivetrainSubsystem driveTrain, VisionTracking visionTracking)
    {
        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
    }

    @Override
    public void execute()
    {   
        if (wheelReset == true){
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0));
            m_visionTracking.setLED(0);
            wheelReset = false;
        }
        // get a reference to the subtable called "datatable"
        double tx = m_visionTracking.getX();

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, tx));
        //wheelAngle += 1; 

        }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        wheelAngle = 0;
        wheelReset = true;
        m_visionTracking.setLED(1);
    }
}

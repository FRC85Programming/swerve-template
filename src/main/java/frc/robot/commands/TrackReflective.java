package frc.robot.commands;

import javax.swing.text.AbstractDocument.BranchElement;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;

public class TrackReflective extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;
    //private final Pigeon2 m_pigeon;
    boolean left = false;
    boolean lrDecided = false;
    public TrackReflective(DrivetrainSubsystem driveTrain, VisionTracking visionTracking){

        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
        //m_pigeon = pigeon2;
   }


    @Override
    public void execute() {
        // Sets the limelight LEDs to "Force on"
        double tx = m_visionTracking.getX();
        double area = m_visionTracking.getArea();
        // Grabs the tapes distance from the crosshair (tx) and the area the tape takes up on the screen (area) as a percentage
        if (area >= 1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0.0, -tx * area * .08));
            // Makes the robot drive slower when the tape is closer
        }
         else if (area < 1 && area >= .1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0.0, -tx * area * 2));
            // Makes the robot drive a little faster when the tape is medium distance
         }
        else if (area < .1 && area > 0) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0.0, -tx * area * 11));
            // Makes the robot drive faster when the tape is very far away
        } 
    }

    @Override
    public void end (boolean interrupted)  {
        m_visionTracking.setLED(1);
        // Sets limelight LEDs to "Force off"
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        lrDecided = false;
        // Tells the robot to stop moving even if it detects someone
    }
}
 

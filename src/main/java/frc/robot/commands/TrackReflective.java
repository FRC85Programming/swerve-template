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
        m_visionTracking.setLED(0);
        m_visionTracking.setPipeline(1);
        double tx = m_visionTracking.getX();
        double area = m_visionTracking.getArea();
        double thor = m_visionTracking.getThor();
        //double yaw = m_pigeon.getYaw();
        if (area >= 1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0, -tx * area * .08));
        }
        else if (area < 1 && area >= .1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0, -tx * area * 2));
        } 
        else if (area < .1 && area > 0) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0, -tx * area * 11));
        } 
    }



    @Override
    public void end(boolean interrupted)  {
        m_visionTracking.setLED(1);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        lrDecided = false;
    }
}

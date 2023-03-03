package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLineup extends CommandBase
{

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public Boolean driveDone;
    public Boolean track;
    public Boolean trackDone;
    private final VisionTracking vision;
    Timer m_timer;
    Boolean timerStarted = false;
    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision) {
        m_drivetrainSubsystem = driveTrain;
        this.vision = vision;
        trackDone = false;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        vision.setPipeline(2);
        double tx = vision.getX();
        double area = vision.getArea();
        //One encoder tic = 2.75 feet
        // Drives the robot given the specified values
        if (trackDone == false) {
            SmartDashboard.putNumber("area", area);
            SmartDashboard.putNumber("tx", tx);
            SmartDashboard.putNumber("target tx", 19.5 * area + 1.7);
            if (tx - 2 < 19.5 * area + 1.7 && tx + 2 > 19.5 * area + 2){
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
                trackDone = true;
            } else if (tx - 2 > 19.5 * area + 1.7) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0.5));
            } else if (tx + 2 < 19.5 * area + 1.7) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -0.5));
            } 
        }
    }
    @Override
    public boolean isFinished(){
        return trackDone == true;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        trackDone = false;
        m_timer.reset();
    }
}

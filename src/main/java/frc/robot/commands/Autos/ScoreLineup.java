package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLineup extends CommandBase
{

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public Boolean driveDone;
    public Boolean track;
    public Boolean trackDone;
    private final VisionTracking vision;
    Boolean timerStarted = false;
    private double areaValue;
    private Boolean areaChecked = false;
    private double tx;
    private double area;
    boolean driveLeftStarted = false;
    boolean driveRightStarted = false;
    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision) {
        m_drivetrainSubsystem = driveTrain;
        this.vision = vision;
        trackDone = false;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        vision.setPipeline(2);
        tx = vision.getX();
        area = vision.getArea();

        if (areaChecked == false) {
            areaValue = area;
            areaChecked = true;
        }

        if (areaValue == 0) {
            while (true) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, 0, 0));
            }
        }

        if (tx > -3.441302*areaValue-3.473436) {
            if (driveLeftStarted != true) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, -0.5, 0));
                driveRightStarted = true;
            }
        }
        if (tx < -3.441302*areaValue-3.473436) {
            if (driveRightStarted != true) { }
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.5, 0.5, 0));
                driveLeftStarted = true;
        }
    }


    @Override
    public boolean isFinished(){
        if (driveLeftStarted == true) {
            return tx < -3.441302*areaValue-3.473436;
        }
        if (driveRightStarted == true) {
            return tx > -3.441302*areaValue-3.473436;
        } else {
            return false;
        }
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        trackDone = false;
        areaChecked = false;
        driveLeftStarted = false;
        driveRightStarted = false;
    }
}

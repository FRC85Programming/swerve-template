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
    Timer m_timer;
    Boolean timerStarted = false;
    private double areaValue;
    private Boolean areaChecked = false;
    private double tx;
    private double area;
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
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
            }
        }

        if (tx > -3.441302*areaValue-3.473436) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.5, 0, 0));
        }
        if (tx < -3.441302*areaValue-3.473436) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.5, 0, 0));
        }

    }
    public Boolean lineupDone(){
        return tx - 0.2 > -3.441302*areaValue-3.473436 && tx + 0.2 < -3.441302*areaValue-3.473436;
    }

    @Override
    public boolean isFinished(){
        return lineupDone();
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        trackDone = false;
        areaChecked = false;
        m_timer.reset();
    }
}

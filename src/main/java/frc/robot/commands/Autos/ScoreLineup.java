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
    private double turnMultiply = 1;
    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision) {
        m_drivetrainSubsystem = driveTrain;
        this.vision = vision;
        trackDone = false;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        vision.setPipeline(2);
        vision.setLED(0);
        tx = vision.getX();
        area = vision.getArea();

        if (areaChecked == false) {
            areaValue = area;
            areaChecked = true;
        }
        if (tx-1 > -3.441302*areaValue-3.473436 && tx +1 < -3.441302*areaValue-3.473436) {
            turnMultiply = 0;
        }
        if (tx < -3.441302*areaValue-3.473436) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnMultiply*-0.7));
        } else if (tx < -3.441302*areaValue-3.473436) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnMultiply*0.7));
        }
    }


    @Override
    public boolean isFinished(){
       return turnMultiply == 0 || tx-1 > -3.441302*areaValue-3.473436 && tx +1 < -3.441302*areaValue-3.473436;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        areaChecked = false;
        vision.setLED(1);
    }
}

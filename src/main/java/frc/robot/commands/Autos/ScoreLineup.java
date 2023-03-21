package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        PIDController lineup = new PIDController(1, 0, 0);
        //One encoder tic = 2.75 feet
        // Drives the robot given the specified values
        if (trackDone == false) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, lineup.calculate(tx - 6, area)));
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

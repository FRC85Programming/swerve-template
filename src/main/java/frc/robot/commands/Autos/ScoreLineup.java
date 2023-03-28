package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLineup extends CommandBase
{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking vision;
    // Variables are created and set to default values
    double tx;
    double area;
    // PID values that adjust the distance from the offset
    double headingError;
    double Kp = 0.1;
    double steering_adjust;
    // Switch boolean that makes sure we only set the pipline once
    boolean lineSwitched = false;

    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision) {
        vision.setLED(0);
        m_drivetrainSubsystem = driveTrain;
        this.vision = vision;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        switchPipeline();
        collectValues();
        setCorrectionValues();
        
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, steering_adjust));
        
    }

    public void collectValues() {
        // Sets the values of the targets position ad size
        tx = vision.getX();
        area = vision.getArea();
    }

    public void setCorrectionValues() {
        // Sets the values that will be driven in proportion to feedback
        headingError = tx;
        steering_adjust = Kp * tx-7.5;
    }

    public void switchPipeline() {
        // This if statement makes sure we only switch the pipline once so our camera doesn't lag
        if (lineSwitched == false) {
            vision.setPipeline(2);
            lineSwitched = true;
        }
    }


    @Override
    public boolean isFinished(){ 
        // Latency window of x to be lined up
       return headingError-0.5 < 0 && headingError+0.5 > 0;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        lineSwitched = false;
        vision.setLED(1);
    }
}

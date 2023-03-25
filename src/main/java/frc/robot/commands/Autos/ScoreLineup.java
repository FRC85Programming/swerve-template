package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLineup extends CommandBase
{
    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision) {
        createSystems();
        init();
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

    public void init() {
        // Variables are created and set to default values
        private double tx;
        private double area;
        // PID values that adjust the distance from the offset
        double headingError;
        double Kp = 0.1f;
        double steering_adjust;
        // Switch boolean that makes sure we only set the pipline once
        double lineSwitched = false;
        vision.setLED(0);
    }
    
    public void createSystems() {
        // Our two subsystems for this are set up. Vision for detecting cubes and Drivetrain to correct our rotation
        private final DrivetrainSubsystem m_drivetrainSubsystem;
        private final VisionTracking vision;
    }

    public void collectValues() {
        // Sets the values of the targets position ad size
        tx = vision.getX();
        area = vision.getArea();
    }

    public void setCorrectionValues() {
        // Sets the values that will be driven in proportion to feedback
        headingError = tx;
        steering_adjust = Kp * tx;
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
       return headingError-0.2 < 0 && headingError+0.2 > 0;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        lineSwitched = false;
        vision.setLED(1);
    }
}

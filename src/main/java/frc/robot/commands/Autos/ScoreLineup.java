package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLineup extends CommandBase
{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking vision;
    private final RobotContainer robotContainer;
    // Variables are created and set to default values
    double tx;
    double area;
    // PID values that adjust the distance from the offset
    double headingError;
    double Kp = -0.05;
    double steering_adjust;
    // Switch boolean that makes sure we only set the pipline once
    boolean lineSwitched = false;
    double leftStickY;
    boolean endWhenFound;

    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, boolean endWhenFound) {
        vision.setLED(0);
        m_drivetrainSubsystem = driveTrain;
        this.vision = vision;
        this.robotContainer = robotContainer;
        this.endWhenFound = endWhenFound;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        switchPipeline();
        collectValues();
        setCorrectionValues();
        if (robotContainer!= null) {
            leftStickY = robotContainer.getLeftY();
        }

        if (tx != 0) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(leftStickY, 0, steering_adjust));
        } else {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        }

        
    }

    public void collectValues() {
        // Sets the values of the targets position ad size
        tx = vision.getX();
        area = vision.getArea();
    }

    public void setCorrectionValues() {
        // Sets the values that will be driven in proportion to feedback
        headingError = tx;
        steering_adjust = Kp * (tx);
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
        if (tx == 0) {
            return false;
        } else if (endWhenFound == true) {
            // Latency window of x to be lined up
            return headingError - 1.4 < 0 && headingError + 1.4 > 0;
        } else {
            return false;
        }
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        lineSwitched = false;
        vision.setLED(1);
    }
}

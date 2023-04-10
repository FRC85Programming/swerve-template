package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreLineup extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking vision;
    private final RobotContainer robotContainer;
    // Variables are created and set to default values
    double tx1;
    double area1;
    double tx2;
    double area2;
    // PID values that adjust the distance from the offset
    double headingError;
    double Kp = -0.05;
    double steering_adjust;
    // Switch boolean that makes sure we only set the pipline once
    boolean lineSwitched = false;
    double leftStickY;
    boolean endWhenFound;
    boolean leftCamUse = false;
    boolean rightCamUse = false;

    public ScoreLineup(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer,
            boolean endWhenFound) {
        m_drivetrainSubsystem = driveTrain;
        this.vision = vision;
        this.robotContainer = robotContainer;
        this.endWhenFound = endWhenFound;
        addRequirements(m_drivetrainSubsystem);
        addRequirements(vision);
    }

    @Override
    public void execute() {    
        SmartDashboard.putNumber("Auto Heading Error", headingError);
        SmartDashboard.putBoolean("Auto IsFinished", isFinished());
        vision.setLED1(0);
        vision.setLED2(0);
        switchPipeline();
        collectValues();
        setCorrectionValues();

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, steering_adjust));
    }

    public void collectValues() {
        // Sets the values of the targets position ad size
        tx1 = vision.getX1();
        area1 = vision.getArea1();
        tx2 = vision.getX2();
        area2 = vision.getArea2();
    }

    public void setCorrectionValues() {
        // Sets the values that will be driven in proportion to feedback
        headingError = tx1; 

        steering_adjust = Kp * (headingError);
    }

    public void switchPipeline() {
        // This if statement makes sure we only switch the pipline once so our camera
        // doesn't lag
        if (lineSwitched == false) {
            vision.setPipeline1(2);
            vision.setPipeline2(2);
            lineSwitched = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (endWhenFound == true) {
            // Latency window of x to be lined up
            return 0 < Math.abs(headingError) && Math.abs(headingError) < 2.5;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        lineSwitched = false;
        vision.setLED1(1);
        vision.setLED2(1);
        leftCamUse = false;
        rightCamUse = false;
    }
}

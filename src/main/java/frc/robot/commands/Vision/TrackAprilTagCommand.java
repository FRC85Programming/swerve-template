package frc.robot.commands.Vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class TrackAprilTagCommand extends CommandBase {
    // V A R I A B L E S
    double wheelAngle = 0.0;
    boolean wheelReset = true;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;

    public TrackAprilTagCommand(DrivetrainSubsystem driveTrain, VisionTracking visionTracking) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
    }

    @Override
    public void execute() {
        m_visionTracking.setPipeline1(0);
        if (wheelReset == true) {
            // Stops the robot so that nobody dies
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0));
            // Make sure Ashley isn't arounf when this happens... (It turns the Limelight
            // on)
            m_visionTracking.setLED1(0);
            // Switch variable so that this happens once per command run
            wheelReset = false;
        }
        // Sets the variables of the targets x, area, and ID
        double tx = m_visionTracking.getX1();
        double area = m_visionTracking.getArea1();
        double tagID = m_visionTracking.getTag1();
        if (tagID > -1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-1.5 * area + 5, 0.0, -tx * 0.2)); // Max speed 3: 1, 5: 0.6,
                                                                                             // 8: 0.3, 12: 0.1
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stops the bot and changes the switch variables back to the original position
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        wheelAngle = 0;
        wheelReset = true;
        // Makes Ashley not complain. (Turns Limelight off)
        m_visionTracking.setLED1(1);
    }
}

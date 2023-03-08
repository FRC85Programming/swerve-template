package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ActionOnTagCommand extends CommandBase {
    // V A R I A B L E S
    double wheelAngle = 0.0;
    boolean wheelReset = true;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;

    public ActionOnTagCommand(DrivetrainSubsystem driveTrain, VisionTracking visionTracking) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
    }

    @Override
    public void execute() {

        if (wheelReset == true) {
            // Stops the robot so that nobody dies
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0));
            // Make sure Ashley isn't arounf when this happens... (It turns the Limelight
            // on)
            m_visionTracking.setLED(0);
            // Switch variable so that this happens once per command run
            wheelReset = false;
        }
        // Sets the variables of the targets x, area, and ID
        double tx = m_visionTracking.getX();
        double area = m_visionTracking.getArea();
        double tagID = m_visionTracking.getTag();
        if (tagID == 4) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 1, 0.0));
        }
        if (tagID == 3) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, -1, 0.0));
        }
        if (tagID == 5) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0.0, 1.0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stops the bot and changes the switch variables back to the original position
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        wheelAngle = 0;
        wheelReset = true;
        // Makes Ashley not complain. (Turns Limelight off)
        m_visionTracking.setLED(1);
    }
}

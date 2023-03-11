package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LineUpAprilTagCommand extends CommandBase {
    // V A R I A B L E S 
    double wheelAngle = 0.0;
    boolean wheelReset = true;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;

    /**
     * Creates a new LineUpAprilTagCommand
     * @param driveTrain passes in the drive train
     * @param visionTracking passes in the vision tracking
     */
    public LineUpAprilTagCommand(DrivetrainSubsystem driveTrain, VisionTracking visionTracking) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
    }

    /**
     * Runs the command after it's been constructed
     */
    @Override
    public void execute() {
        m_visionTracking.setPipeline(0);
        if (wheelReset == true) {
            // Stops the robot so that nobody dies
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0));
            // Make sure Ashley isn't around when this happens... (It turns the Limelight on)
            m_visionTracking.setLED(0);
            // Switch variable so that this happens once per command run
            wheelReset = false;
        }
        //Sets the variables of the targets x, area, and ID
        double tx = m_visionTracking.getX();
        double area = m_visionTracking.getArea();
        double tagID = m_visionTracking.getTag();
        double length = m_visionTracking.getLength();
        double height = m_visionTracking.getHeight();
        double ty = m_visionTracking.getY();

        if (tagID > -1) {
            if(ty < -0.5)
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, -1.5 * area + 5, 0.0)); // Max speed 3: 1, 5: 0.6, 8: 0.3, 12: 0.1
            else if(ty > 0.5) 
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 1.5 * area + 5, 0.0));
        }
    }

    /**
     * Ends the command
     */
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
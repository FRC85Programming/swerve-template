// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.*;

public class TrackGamePiece extends CommandBase {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final VisionTracking m_visionTracking;

  /** Creates a new TrackGamePieces. */
  public TrackGamePiece(DrivetrainSubsystem driveTrain, VisionTracking visionTracking) {
      // Sets up variables for each subsystem
      m_drivetrainSubsystem = driveTrain;
      m_visionTracking = visionTracking;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_visionTracking.setLED(0);
        // Sets the limelight LEDs to "Force on"
        double tx = m_visionTracking.getX();
        double area = m_visionTracking.getArea();
        // Grabs the game piece's distance from the crosshair (tx) and the area it takes up on the screen (area) as a percentage
        if (area >= 1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0.0, -tx * area * .08));
            // Makes the robot drive slower when the tape is closer
        }
         else if (area < 1 && area >= .1) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0.0, -tx * area * 2));
            // Makes the robot drive a little faster when the tape is medium distance
         }
        else if (area < .1 && area > 0) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(3, 0.0, -tx * area * 11));
            // Makes the robot drive faster when the tape is very far away
        } 
        else {} // Stops the robot if it detects nothing
  }

  // Called once the command ends or is interrupted.
  @Override
    public void end(boolean interrupted)  {
        m_visionTracking.setLED(1);
        // Sets limelight LEDs to "Force off"
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        // Tells the robot to stop moving even if it detects someone
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

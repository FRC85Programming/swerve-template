package frc.robot.commands.Vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;

public class AutoLineup extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionTracking m_visionTracking;
    // private final Pigeon2 m_pigeon;
    boolean left = false;
    boolean sideSelected = false;
    double tagFinal = -1;

    public AutoLineup(DrivetrainSubsystem driveTrain, VisionTracking visionTracking) {

        m_drivetrainSubsystem = driveTrain;
        m_visionTracking = visionTracking;
    }

    @Override
    public void execute() {
        m_visionTracking.setLED(0);
        m_visionTracking.setPipeline(1);
        double tx = m_visionTracking.getX();
        double area = m_visionTracking.getArea();
        double thor = m_visionTracking.getThor();
        double tagID = m_visionTracking.getTag();
        // double yaw = m_pigeon.getYaw();
        if (sideSelected == false) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -tx));
            if (tx < 3 && tx > -3) {
                tagFinal = tagID;
                sideSelected = false;
            }
        }
        if (tagID == 1) {
            if (area >= 1) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(3, -thor - 14, -tx * area * .08));
            } else if (area < 1 && area >= .1) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(3, -thor - 14, -tx * area * 2));
            } else if (area < .1 && area > 0) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(3, -thor - 14, -tx * area * 11));
            }
        }
        if (tagID == 2) {
            if (area >= 1) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(3, -thor + 14, -tx * area * .08));
            } else if (area < 1 && area >= .1) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(3, -thor + 14, -tx * area * 2));
            } else if (area < .1 && area > 0) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(3, -thor + 14, -tx * area * 11));
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_visionTracking.setLED(1);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        sideSelected = false;
    }
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase
 {
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public final double wheelSpeed;
    public final double wheelAngle;
    public Drive(DrivetrainSubsystem driveTrain, double speed, double angle) {
        // Sets up variables for the command
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        wheelSpeed = speed;
        wheelAngle = angle;
    }

    @Override
    public void execute() {

        // Drives each wheel to the set angle and speed
        m_frontLeftModule.set(wheelSpeed, wheelAngle);
        m_frontRightModule.set(wheelSpeed, -wheelAngle);
        m_backLeftModule.set(wheelSpeed, -wheelAngle);
        m_backRightModule.set(wheelSpeed, wheelAngle);
    }

    @Override
    public void end (boolean interrupted)  {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
    }
}

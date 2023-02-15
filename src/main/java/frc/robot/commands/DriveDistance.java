package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase
 {
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public final double wheelSpeed;
    public final double wheelAngle;
    double constantBRDistance;
    double constantBLDistance;
    double constantFLDistance;
    double constantFRDistance;
    Boolean constantCalc;
    double brTarget;
    double blTarget;
    double flTarget;
    double frTarget;
    public DriveDistance(DrivetrainSubsystem driveTrain, double speed, double angle) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        wheelSpeed = speed;
        wheelAngle = angle;
        constantCalc = false;
    }

    @Override
    public void execute() {
        // Gets the drive distance so that we can accuratley judge how far we need to drive
        if (constantCalc == false) {
            constantFLDistance = m_frontLeftModule.getDriveDistance();
            constantFRDistance = m_frontRightModule.getDriveDistance();
            constantBLDistance = m_backLeftModule.getDriveDistance();
            constantBRDistance = m_backRightModule.getDriveDistance();
            flTarget = constantFLDistance + 30;
            frTarget = constantFLDistance + 500;
            blTarget = constantFLDistance + 500;
            brTarget = constantFLDistance + 500;
            constantCalc = true;
        }
        if (m_frontLeftModule.getDriveDistance() - flTarget >= -2 && m_frontLeftModule.getDriveDistance() - flTarget <= 2){
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        } else {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeed, 0, wheelAngle));
        }
        SmartDashboard.putNumber("Encoder FL", m_frontLeftModule.getDriveDistance());
    }

    @Override
    public void end (boolean interrupted)  {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
    }
}

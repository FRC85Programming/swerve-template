package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase
{
    private final SwerveModule m_frontLeftModule;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double wheelSpeed;
    private double wheelAngle;
    private double encoderTarget;
    private double constantFLDistance;
    private Boolean constantCalc;
    private double flTarget;
    public DriveDistance(DrivetrainSubsystem driveTrain, double speed, double angle, double target) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        wheelSpeed = speed;
        wheelAngle = angle;
        encoderTarget = target;
        constantCalc = false;
    }

    @Override
    public void execute() {
        // Gets the drive distance so that we can accuratley judge how far we need to drive
        if (constantCalc == false) {
            constantFLDistance = m_frontLeftModule.getDriveDistance();
            // Sample equation  target = 10.5+5, target = 15.5, 5 more than the first value assuming the specified distance is 5
            flTarget = constantFLDistance + encoderTarget;
            constantCalc = true;
        }
        //One encoder tic = 2.75 feet
        // Drives the robot given the specified values
        m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeed, 0, wheelAngle));
        SmartDashboard.putNumber("Encoder FL", m_frontLeftModule.getDriveDistance());
    }

    @Override
    public boolean isFinished(){
        // Ends the command when it hits the target encoder distance
        return m_frontLeftModule.getDriveDistance() - flTarget >= -0.3 && m_frontLeftModule.getDriveDistance() - flTarget <= 0.3;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        constantCalc = false;
    }
}

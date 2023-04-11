package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SideDependentStrafe extends CommandBase
{
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double wheelSpeedX;
    private double wheelSpeedY;
    private double encoderTarget;
    private Boolean constantCalc;
    private Boolean encoderResetDone;
    private static int instanceCount = 0;
    private boolean driveDone = false;
    Boolean timerStarted = false;
    DriverStation.Alliance color;
    double degrees360;
    double avgEncoderDistance = 0;
    public SideDependentStrafe(DrivetrainSubsystem driveTrain, double driveTarget) {
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        encoderTarget = driveTarget;
        init();
        addRequirements(m_drivetrainSubsystem);
    }

    private void init() {
        // Sets up variables for each subsystem
        constantCalc = false;
        encoderResetDone = false;
        driveDone = false;
        instanceCount++;
        SmartDashboard.putNumber("instance count", instanceCount);
    }

    @Override
    public void execute() {
        if (constantCalc == false) {
            m_drivetrainSubsystem.resetDriveEncoders();
            //constantDistance = avgEncoderDistance;
            // Sample equation  target = 10.5+5, target = 15.5, 5 more than the first value assuming the specified distance is 5
            constantCalc = true;
        }

 
        color = DriverStation.getAlliance();
        avgEncoderDistance = (Math.abs(m_backLeftModule.getDriveDistance()) + Math.abs(m_backRightModule.getDriveDistance()) + Math.abs(m_frontLeftModule.getDriveDistance()) + Math.abs(m_frontRightModule.getDriveDistance())) / 4;
        if (!encoderResetDone) {
            if (avgEncoderDistance != 0) {
                return;
            }

            encoderResetDone = true;
        }

        degrees360 = (m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() +360)% 360;
        SmartDashboard.putNumber("auto Target", encoderTarget);
        SmartDashboard.putNumber("auto Avg Encoder Distance", avgEncoderDistance);
        SmartDashboard.putNumber("auto Front Left encoder", m_frontLeftModule.getDriveDistance());
        SmartDashboard.putNumber("auto Front Right encoder", m_frontRightModule.getDriveDistance());
        SmartDashboard.putNumber("auto Back Left encoder", m_backLeftModule.getDriveDistance());
        SmartDashboard.putNumber("auto Back Right encoder", m_backRightModule.getDriveDistance());
        if (timerStarted == false) {
            timerStarted = true;
        }
        //One encoder tic = 2.75 feet (old)
        // Makes sure the robot stops when drive is done
        if (driveFinished()) {
            wheelSpeedX = 0;
            wheelSpeedY = 0;
            driveDone = true;
        }
        SmartDashboard.putBoolean("DriveFinished", driveFinished());
        //Aliance based strafe
        if (driveFinished() == false) {
            if (wheelSpeedY != 0 && wheelSpeedX != 0) {
                if (color == DriverStation.Alliance.Blue) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, 1, 0));
                } else if (color == DriverStation.Alliance.Red) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, -1, 0));
                }
            }
        }
            
    }
    private boolean driveFinished() {
        return constantCalc && Math.abs(avgEncoderDistance) >= Math.abs(encoderTarget);
    }

    @Override
    public boolean isFinished(){
        return driveDone == true;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        init();
    }
}
package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase
{
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double wheelSpeedX;
    private double wheelSpeedY;
    private double turnSpeed;
    private double encoderTarget;
    private Boolean constantCalc;
    private Boolean encoderResetDone;
    private double targetAngle;
    private boolean angleCalc;
    private double angleTarget;
    private Boolean turnDone;
    public boolean driveDone;
    public Boolean track;
    public Boolean trackDone;
    private static int instanceCount = 0;
    Boolean timerStarted = false;
    DriverStation.Alliance color;
    double degrees360;
    double avgEncoderDistance = 0;
    public DriveDistance(DrivetrainSubsystem driveTrain, VisionTracking vision, double speedY, double speedX, double rotateSpeed, double driveTarget, double angleTarget, Boolean track) {
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        this.track = track;
        wheelSpeedX = speedX;
        wheelSpeedY = speedY;
        turnSpeed = rotateSpeed;
        encoderTarget = driveTarget;
        this.angleTarget = angleTarget;
        init();
        addRequirements(m_drivetrainSubsystem);
    }

    private void init() {
        // Sets up variables for each subsystem
        constantCalc = false;
        encoderResetDone = false;
        angleCalc = false;
        turnDone = false;
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

        if (turnSpeed != 0) {
            if (angleCalc == false) {
                targetAngle = (degrees360 + angleTarget) % 360;
                angleCalc = true;
            }
        }
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
        if (turnFinished()) {
            turnSpeed = 0;
            turnDone = true;
        }
        SmartDashboard.putNumber("Drive Distance", m_frontLeftModule.getDriveDistance());
        SmartDashboard.putBoolean("DriveFinished", driveFinished());
        //Aliance based strafe
        if (driveFinished() == false) {
            if (wheelSpeedY != 0 && wheelSpeedX != 0) {
                if (color == DriverStation.Alliance.Blue) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, -1, 0));
                } else if (color == DriverStation.Alliance.Red) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, 1, 0));
                }
            }

                // If more than 3/4 of the wat through the drive, start rampdown
                m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, turnSpeed));
                if (encoderTarget/4 <  encoderTarget - avgEncoderDistance) {
                    m_drivetrainSubsystem.setOpenloopRate(0);
                } else {
                    m_drivetrainSubsystem.setOpenloopRate(1);
                }
        }
        if (turnFinished() == false) {
            // Only rotate if driveforward and side speeds are zero
            if (wheelSpeedX == 0 && wheelSpeedY == 0) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnSpeed));
            } 
        }
            
    }
    private boolean driveFinished() {
        return constantCalc && Math.abs(avgEncoderDistance) >= Math.abs(encoderTarget);
    }
    private boolean turnFinished() {
        return turnSpeed == 0 || degrees360 - targetAngle >= -3 && degrees360 - targetAngle <= 3;
    }

    @Override
    public boolean isFinished(){
        return turnDone == true && driveDone == true;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        m_drivetrainSubsystem.setOpenloopRate(0);
        init();
    }
}
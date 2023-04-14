package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    private Boolean encoderResetTrigger;
    private Boolean encoderResetDone;
    private double targetAngle;
    private boolean angleCalc;
    private double angleTarget;
    private Boolean turnDone;
    public boolean driveDone = false;
    public Boolean track;
    public Boolean trackDone;
    private static int instanceCount = 0;
    DriverStation.Alliance color;
    double degrees360;
    double startDegrees;
    double avgEncoderDistance = 0;
    Timer m_timer;
    Boolean useRamp;
    public DriveDistance(DrivetrainSubsystem driveTrain, VisionTracking vision, double speedY, double speedX, double rotateSpeed, double driveTarget, double angleTarget, Boolean useRamp) {
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        this.useRamp = useRamp;
        wheelSpeedX = speedX;
        wheelSpeedY = speedY;
        turnSpeed = rotateSpeed;
        encoderTarget = driveTarget;
        this.angleTarget = angleTarget;
        init();
        addRequirements(m_drivetrainSubsystem);        
        instanceCount++;
        SmartDashboard.putNumber("instance count", instanceCount);
        m_timer = new Timer();
    }

    private void init() {
        // Sets up variables for each subsystem
        encoderResetTrigger = false;
        encoderResetDone = false;
        angleCalc = false;
        turnDone = false;
        driveDone = false;
    }

    @Override
    public void execute() {
        if (encoderResetTrigger == false) {
            DriverStation.reportWarning("Reseting Encoders", false);
            m_drivetrainSubsystem.resetDriveEncoders();
            m_timer.reset();
            m_timer.start();
            // Sample equation  target = 10.5+5, target = 15.5, 5 more than the first value assuming the specified distance is 5
            encoderResetTrigger = true;
        }
 
        avgEncoderDistance = (Math.abs(m_backLeftModule.getDriveDistance()) + Math.abs(m_backRightModule.getDriveDistance()) + Math.abs(m_frontLeftModule.getDriveDistance()) + Math.abs(m_frontRightModule.getDriveDistance())) / 4;
        degrees360 = (m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() +360)% 360;
        
        if (!encoderResetDone) {
            if (avgEncoderDistance > 0.05) {
                DriverStation.reportWarning("Encoder Position Average: " + avgEncoderDistance, false);
                return;
            }
            DriverStation.reportWarning("Encoders Reset", false);
            encoderResetDone = true;
            startDegrees = degrees360;
        }

        SmartDashboard.putNumber("auto Target", encoderTarget);
        SmartDashboard.putNumber("auto Avg Encoder Distance", avgEncoderDistance);
        SmartDashboard.putNumber("auto Angle Target", targetAngle);
        SmartDashboard.putNumber("auto Current Rotation", degrees360);
        SmartDashboard.putNumber("auto Front Left encoder", m_frontLeftModule.getDriveDistance());
        SmartDashboard.putNumber("auto Front Right encoder", m_frontRightModule.getDriveDistance());
        SmartDashboard.putNumber("auto Back Left encoder", m_backLeftModule.getDriveDistance());
        SmartDashboard.putNumber("auto Back Right encoder", m_backRightModule.getDriveDistance());

        if (turnSpeed != 0) {
            if (angleCalc == false) {
                DriverStation.reportWarning("Calculating target angle", false);
                targetAngle = (degrees360 + angleTarget) % 360;
                angleCalc = true;
            }
        }
        //One encoder tic = 2.75 feet (old)
        // Makes sure the robot stops when drive is done
        if (driveFinished()) {
            DriverStation.reportWarning("Drive to " + encoderTarget + " Finished", false);
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
            driveDone = true;
        }
        if (turnFinished()) {
            DriverStation.reportWarning("Turn Finished", false);
            turnDone = true;
        }
        SmartDashboard.putNumber("Drive Distance", m_frontLeftModule.getDriveDistance());
        SmartDashboard.putBoolean("DriveFinished", driveFinished());

        if (!driveDone && (wheelSpeedX != 0 || wheelSpeedY != 0)) {
            double correctionTurnSpeed = 0;
            if (Math.abs(wheelSpeedX) > 0.7 || Math.abs(wheelSpeedY) > 0.7) {
                if (degrees360 > startDegrees + 1) {
                    correctionTurnSpeed = -0.3;
                } else if (degrees360 < startDegrees - 1) {
                    correctionTurnSpeed = 0.3;
                }
            }
            
            DriverStation.reportWarning("Driving (" + wheelSpeedX + ", " + wheelSpeedY + ") with correction " + correctionTurnSpeed, false);
            // If first 10% or more than 3/4 of the way through the drive, start rampdown
            if (useRamp == true) {
                if (wheelSpeedX > 0) {
                    if (m_frontLeftModule.getDriveVelocity() < wheelSpeedX) {
                        m_drivetrainSubsystem.drive(new ChassisSpeeds(m_timer.get()*0.75+.75, wheelSpeedY, correctionTurnSpeed));
                    }
                    if (m_frontLeftModule.getDriveVelocity() >= wheelSpeedX) {
                        m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, correctionTurnSpeed));
                    }
                }
                if (wheelSpeedX < 0) {
                    if (m_frontLeftModule.getDriveVelocity() > wheelSpeedX) {
                        m_drivetrainSubsystem.drive(new ChassisSpeeds(m_timer.get()*-0.75-.75, wheelSpeedY, correctionTurnSpeed));
                    }
                    if (m_frontLeftModule.getDriveVelocity() <= wheelSpeedX) {
                        m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, correctionTurnSpeed));
                    }
                }
            } else {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, correctionTurnSpeed));
            }
            // if (encoderTarget / 4 > encoderTarget - avgEncoderDistance || 0.9 * encoderTarget < encoderTarget - avgEncoderDistance) {
            //     m_drivetrainSubsystem.setOpenloopRate(0.8);
            // } else {
            //     m_drivetrainSubsystem.setOpenloopRate(0);
            // }
        }
        
        if (turnFinished() == false) {
            // Only rotate if driveforward and side speeds are zero
            if (wheelSpeedX == 0 && wheelSpeedY == 0) {
                DriverStation.reportWarning("Turning", false);
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnSpeed));
            } 
        }
            
    }
    private boolean driveFinished() {
        return encoderResetDone && Math.abs(avgEncoderDistance) >= Math.abs(encoderTarget);
    }
    private boolean turnFinished() {
        return turnDone || turnSpeed == 0 || degrees360 - targetAngle >= -2 && degrees360 - targetAngle <= 2;
    }

    @Override
    public boolean isFinished(){
        return turnDone == true && driveDone == true && encoderResetDone;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        DriverStation.reportWarning("DriveDistance command end", false);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        m_drivetrainSubsystem.setOpenloopRate(0);
        m_timer.reset();
    }
}
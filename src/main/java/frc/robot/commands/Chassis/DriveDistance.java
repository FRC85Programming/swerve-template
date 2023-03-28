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
    private double constantDistance;
    private Boolean constantCalc;
    private double tickTarget;
    private double tickTargetMinus;
    private double targetAngle;
    private boolean angleCalc;
    private double angleTarget;
    private Boolean turnDone;
    public Boolean driveDone;
    public Boolean track;
    public Boolean trackDone;
    private static int instanceCount = 0;
    Boolean timerStarted = false;
    DriverStation.Alliance color;
    double degrees360;
    double avgEncoderDistance;
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
        angleCalc = false;
        turnDone = false;
        driveDone = false;
        instanceCount++;
        SmartDashboard.putNumber("instance count", instanceCount);
    }

    @Override
    public void execute() {
        color = DriverStation.getAlliance();
        avgEncoderDistance = (m_backLeftModule.getDriveDistance() + m_backRightModule.getDriveDistance() + m_frontLeftModule.getDriveDistance() + m_frontRightModule.getDriveDistance()) / 4;
        degrees360 = (m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() +360)% 360;
        if (turnSpeed != 0) {
            if (angleCalc == false) {
                targetAngle = degrees360 + angleTarget;
                angleCalc = true;
            }
        }
        if (timerStarted == false) {
            timerStarted = true;
        }
        // Gets the drive distance so that we can accuratley judge how far we need to drive
        if (constantCalc == false) {
            constantDistance = avgEncoderDistance;
            // Sample equation  target = 10.5+5, target = 15.5, 5 more than the first value assuming the specified distance is 5
            tickTarget = constantDistance + encoderTarget;
            constantCalc = true;

            tickTarget = constantDistance - encoderTarget;
        }
        //One encoder tic = 2.75 feet
        // Drives the robot given the specified values
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
        if (driveFinished() == false) {
            if (wheelSpeedY != 0 && wheelSpeedX != 0) {
                if (color == DriverStation.Alliance.Blue) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, -1, 0));
                } else if (color == DriverStation.Alliance.Red) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-1, 1, 0));
                }
            }


                m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, turnSpeed));
                if (tickTarget/4 <  tickTarget - avgEncoderDistance || tickTargetMinus/4 > tickTargetMinus + avgEncoderDistance) {
                    m_drivetrainSubsystem.setOpenloopRate(0);
                } else {
                    m_drivetrainSubsystem.setOpenloopRate(1);
                }
        }
        if (turnFinished() == false) {
            if (wheelSpeedX == 0 && wheelSpeedY == 0) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnSpeed));
            } 
        }
            
    }
    private boolean driveFinished() {
        return avgEncoderDistance- tickTarget >= -0.3 && avgEncoderDistance- tickTarget <= 0.3 || avgEncoderDistance - tickTargetMinus >= -0.3 && avgEncoderDistance - tickTargetMinus <= 0.3;
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
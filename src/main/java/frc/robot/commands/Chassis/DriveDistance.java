package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionTracking;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
    private final SwerveModule m_frontLeftModule;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double wheelSpeedX;
    private double wheelSpeedY;
    private double turnSpeed;
    private double encoderTarget;
    private double constantFLDistance;
    private Boolean constantCalc;
    private double flTarget;
    private double flTargetMinus;
    private int counter = 0;
    private double targetAngle;
    private boolean angleCalc;
    private double angleTarget;
    private Boolean turnDone;
    public Boolean driveDone;
    public Boolean track;
    public Boolean trackDone;
    private static int instanceCount = 0;
    Timer m_timerPositive;
    Timer m_timerNegative;
    Boolean timerStarted = false;
    public DriveDistance(DrivetrainSubsystem driveTrain, VisionTracking vision, double speedY, double speedX, double rotateSpeed, double driveTarget) {
        this(driveTrain, vision, speedY, speedX, rotateSpeed, driveTarget, 0, false);
    }
    public DriveDistance(DrivetrainSubsystem driveTrain, VisionTracking vision, double speedY, double speedX, double rotateSpeed, double driveTarget, double angleTarget, Boolean track) {
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
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
        m_timerPositive = new Timer();
        m_timerNegative = new Timer();
        SmartDashboard.putNumber("instance count", instanceCount);
    }

    @Override
    public void execute() {
        if (turnSpeed != 0) {
            if (angleCalc == false) {
                targetAngle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + angleTarget;
                angleCalc = true;
            }
        }
        if (timerStarted == false) {
            m_timerPositive.start();
            timerStarted = true;
        }
        // Gets the drive distance so that we can accuratley judge how far we need to drive
        SmartDashboard.putNumber("Auto Counter", counter);
        SmartDashboard.putNumber("Fl Speed", m_frontLeftModule.getDriveVelocity());
        if (constantCalc == false) {
            constantFLDistance = m_frontLeftModule.getDriveDistance();
            // Sample equation  target = 10.5+5, target = 15.5, 5 more than the first value assuming the specified distance is 5
            flTarget = constantFLDistance + encoderTarget;
            constantCalc = true;

            flTargetMinus = constantFLDistance - encoderTarget;
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
        SmartDashboard.putNumber("Turn Speed", turnSpeed);
        SmartDashboard.putNumber("targetAngle", targetAngle);
        if (flTarget-m_frontLeftModule.getDriveDistance() < flTarget/4) {
            m_timerNegative.start();
            if (wheelSpeedX > 0) {
            if (m_timerNegative.get() * 0.5 + 0.5 > wheelSpeedX) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, 0, turnSpeed));
            } 
            if (m_timerNegative.get() * 0.5 + 0.5 < wheelSpeedX) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(m_timerNegative.get() *-0.5+2, 0, turnSpeed));
            } 
        }
        if (wheelSpeedX < 0) {
            if (m_timerNegative.get() * -0.5 - 0.5 > wheelSpeedX) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(m_timerNegative.get() *-0.5+2, 0, turnSpeed));
            } 
            if (m_timerNegative.get() * -0.5 - 0.5 < wheelSpeedX) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, 0, turnSpeed));
            } 
        }
         if (wheelSpeedY > 0) {
            if (m_timerNegative.get() * 0.5 - 0.5 > wheelSpeedY) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, m_timerNegative.get()*-0.5+2, turnSpeed));
            } 
            if (m_timerNegative.get() * 0.5 - 0.5 < wheelSpeedY) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, wheelSpeedY, turnSpeed));
            } 
        }
        if (wheelSpeedY < 0) {
            if (m_timerNegative.get() * -0.5 - 0.5 > wheelSpeedY) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, wheelSpeedY, turnSpeed));
            } 
            if (m_timerNegative.get()* -0.5 - 0.5 < wheelSpeedY) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, m_timerNegative.get()*-0.5+2, turnSpeed));
            }
        }
        } else {
            if (wheelSpeedX > 0) {
                if (m_timerPositive.get() * 0.5 + 0.5 > wheelSpeedX) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, 0, turnSpeed));
                } 
                if (m_timerPositive.get() * 0.5 + 0.5 < wheelSpeedX) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(m_timerPositive.get() * 0.5 + 0.5, 0, turnSpeed));
                } 
            }
            if (wheelSpeedX < 0) {
                if (m_timerPositive.get() * -0.5 - 0.5 > wheelSpeedX) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(m_timerPositive.get() * -0.5 - 0.5, 0, turnSpeed));
                } 
                if (m_timerPositive.get() * -0.5 - 0.5 < wheelSpeedX) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, 0, turnSpeed));
                } 
            }
             if (wheelSpeedY > 0) {
                if (m_timerPositive.get() * 0.5 - 0.5 > wheelSpeedY) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, m_timerPositive.get() * -0.5 - 0.5, turnSpeed));
                } 
                if (m_timerPositive.get() * 0.5 - 0.5 < wheelSpeedY) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, wheelSpeedY, turnSpeed));
                } 
            }
            if (wheelSpeedY < 0) {
                if (m_timerPositive.get() * -0.5 - 0.5 > wheelSpeedY) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, wheelSpeedY, turnSpeed));
                } 
                if (m_timerPositive.get() * -0.5 - 0.5 < wheelSpeedY) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, m_timerPositive.get() * -0.5 - 0.5, turnSpeed));
                }
            }
        }

        if (wheelSpeedX == 0 && wheelSpeedY == 0) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnSpeed));
        } 
    
    }
    private boolean driveFinished() {
        return m_frontLeftModule.getDriveDistance() - flTarget >= -0.3 && m_frontLeftModule.getDriveDistance() - flTarget <= 0.3 || m_frontLeftModule.getDriveDistance() - flTargetMinus >= -0.3 && m_frontLeftModule.getDriveDistance() - flTargetMinus <= 0.3;
    }
    private boolean turnFinished() {
        return turnSpeed == 0 || (Math.abs(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()) - targetAngle >= -1 && Math.abs(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()) - targetAngle <= 1);
    }

    @Override
    public boolean isFinished(){
        return turnDone == true && driveDone == true;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        m_timerPositive.reset();
        m_timerNegative.reset();
        init();
    }
}

package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase
{
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
    private double gyroRotation;
    private double targetAngle;
    private boolean angleCalc;
    private double angleTarget;
    private double intakeSpeed;
    private Boolean intakeOn;
    private Boolean turnDone;
    public Boolean driveDone;
    public Boolean track;
    public Boolean trackDone;
    private static int instanceCount = 0;
    private final VisionTracking vision;
    private Boolean switchDone = false;
    private double switchTime = 0;
    Timer m_timer;
    Timer m_rampDownTimer;
    Boolean timerStarted = false;
    public DriveDistance(DrivetrainSubsystem driveTrain, VisionTracking vision, double speedY, double speedX, double rotateSpeed, double driveTarget, double angleTarget, Boolean track) {
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        this.track = track;
        this.vision = vision;
        wheelSpeedX = speedX;
        wheelSpeedY = speedY;
        turnSpeed = rotateSpeed;
        encoderTarget = driveTarget;
        this.angleTarget = angleTarget;
        m_drivetrainSubsystem.setOpenloopRate(1);
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
        m_timer = new Timer();
        m_rampDownTimer = new Timer();
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
            m_timer.start();
            timerStarted = true;
        }
        // Gets the drive distance so that we can accuratley judge how far we need to drive
        SmartDashboard.putNumber("Auto Counter", counter);
        SmartDashboard.putNumber("Fl Speed", m_frontLeftModule.getDriveVelocity());
        PIDController acceleration = new PIDController(.5, 0, .05);
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
        SmartDashboard.putNumber("Drive Distance", m_frontLeftModule.getDriveDistance());
        SmartDashboard.putBoolean("DriveFinished", driveFinished());
        if (driveFinished() == false) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, turnSpeed));
            if (flTarget/4 <  flTarget - m_frontLeftModule.getDriveDistance() || flTargetMinus/4 > flTargetMinus + m_frontLeftModule.getDriveDistance()) {
                m_drivetrainSubsystem.setOpenloopRate(-1.3);
            }
            /*// Forwards and back speed control
            if (wheelSpeedY > 0) {
                if (m_frontLeftModule.getDriveVelocity() < wheelSpeedY) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, -(0.8*m_timer.get()-2)*(0.8*m_timer.get()-2) + wheelSpeedY, angleTarget));
                } else if (m_frontLeftModule.getDriveVelocity() >= wheelSpeedY - 0.2) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedY, 0, 0));
                } 
                if (wheelSpeedY != 0 && flTarget/4 < flTarget-m_frontLeftModule.getDriveDistance()) {
                    m_timer.reset();
                    m_timer.start();
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-(0.6*m_timer.get()) * (0.6*m_timer.get()) + wheelSpeedY, 0, 0));
                }
            } else if (wheelSpeedY < 0) {
                if (m_frontLeftModule.getDriveVelocity() > wheelSpeedY) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, (0.8*m_timer.get()-2)*(0.8*m_timer.get()-2) - wheelSpeedY, angleTarget));
                } else if (m_frontLeftModule.getDriveVelocity() <= wheelSpeedY + 0.2) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedY, 0, 0));
                }
                if (wheelSpeedY != 0 && flTargetMinus/4 < flTargetMinus + -m_frontLeftModule.getDriveDistance()) {
                    m_timer.reset();
                    m_timer.start();
                    m_drivetrainSubsystem.drive(new ChassisSpeeds((0.7*m_timer.get())*(0.7*m_timer.get()) + wheelSpeedY, 0, 0));
                }
            }
            // Left and right speed control
            if (wheelSpeedX > 0) {
                if (m_frontLeftModule.getDriveVelocity() < wheelSpeedX) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, -(0.8*m_timer.get()-2)*(0.8*m_timer.get()-2) + wheelSpeedX, angleTarget));
                } else if (m_frontLeftModule.getDriveVelocity() >= wheelSpeedX - 0.2) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, 0, 0));
                }
                if (wheelSpeedX != 0 && flTarget/4 < flTarget-m_frontLeftModule.getDriveDistance()) {
                    m_timer.reset();
                    m_timer.start();
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(-(0.6*m_timer.get()) * (0.6*m_timer.get()) + wheelSpeedX, 0, 0));
                }
            } else if (wheelSpeedX < 0) {
                if (m_frontLeftModule.getDriveVelocity() > wheelSpeedX) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(0, (0.8*m_timer.get()-2)*(0.8*m_timer.get()-2) - wheelSpeedX, angleTarget));
                } else if (m_frontLeftModule.getDriveVelocity() <= wheelSpeedX + 0.2) {
                    m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, 0, 0));
                }
                if (wheelSpeedY != 0 && flTargetMinus/4 < flTargetMinus + -m_frontLeftModule.getDriveDistance()) {
                    m_timer.reset();
                    m_timer.start();
                    m_drivetrainSubsystem.drive(new ChassisSpeeds((0.7*m_timer.get())*(0.7*m_timer.get()) + wheelSpeedX, 0, 0));
                }
            }*/
            

            if (wheelSpeedX == 0 && wheelSpeedY == 0) {
                m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, turnSpeed));
            } 
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
        switchTime = 0;
        switchDone = false;
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
        m_timer.reset();
        m_rampDownTimer.reset();
        m_drivetrainSubsystem.setOpenloopRate(0);
        init();
    }
}
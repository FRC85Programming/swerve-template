package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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
    public DriveDistance(DrivetrainSubsystem driveTrain, double speedY, double speedX, double rotateSpeed, double driveTarget) {
        this(driveTrain, speedY, speedX, rotateSpeed, driveTarget, 0);
    }
    public DriveDistance(DrivetrainSubsystem driveTrain, double speedY, double speedX, double rotateSpeed, double driveTarget, double angleTarget) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        wheelSpeedX = speedX;
        wheelSpeedY = speedY;
        turnSpeed = rotateSpeed;
        encoderTarget = driveTarget;
        constantCalc = false;
        angleCalc = false;
        this.angleTarget = angleTarget;
        turnDone = false;
        driveDone = false;


        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if (turnSpeed != 0) {
            if (angleCalc == false) {
                targetAngle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() + angleTarget;
                angleCalc = true;
            }
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
        m_drivetrainSubsystem.drive(new ChassisSpeeds(wheelSpeedX, wheelSpeedY, turnSpeed));
        SmartDashboard.putNumber("Rotation", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
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
        constantCalc = false;
        angleCalc = false;
        turnDone = false;
        driveDone = false;
    }
}

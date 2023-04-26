package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PathSwerveSubsystem extends SubsystemBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private Pigeon2 gyro = m_drivetrainSubsystem.getGyro();
    private SwerveDriveOdometry odometer = m_drivetrainSubsystem.getOdo();

    PathSwerveModules flPathModule = new PathSwerveModules(
        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.kFrontLeftDriveEncoderReversed,
        Constants.kFrontLeftTurningEncoderReversed,
        // Find out what absolute encoder ID's are
        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        Constants.FRONT_LEFT_MODULE_STEER_OFFSET,
        Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    PathSwerveModules frPathModule = new PathSwerveModules(
        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.kFrontRightDriveEncoderReversed,
        Constants.kFrontRightTurningEncoderReversed,
        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET,
        Constants.kFrontRightDriveAbsoluteEncoderReversed);

    PathSwerveModules blPathModule = new PathSwerveModules(
        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.kBackLeftDriveEncoderReversed,
        Constants.kBackLeftTurningEncoderReversed,
        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_OFFSET,
        Constants.kBackLeftDriveAbsoluteEncoderReversed);

    PathSwerveModules brPathModule = new PathSwerveModules(
        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.kBackRightDriveEncoderReversed,
        Constants.kBackRightTurningEncoderReversed,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_OFFSET,
        Constants.kBackRightDriveAbsoluteEncoderReversed);

    public PathSwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                m_drivetrainSubsystem.zeroGyroscope(0);
            } catch (Exception e) {
            }
        }).start();
    }


    public double getHeading() {
        return Math.IEEEremainder(gyro.getCompassHeading(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        SwerveModulePosition statesArray[] = new SwerveModulePosition[]{flPathModule.getPosition(), frPathModule.getPosition(), blPathModule.getPosition(), brPathModule.getPosition()};

        odometer.update(getRotation2d(), statesArray);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        flPathModule.stop();
        frPathModule.stop();
        blPathModule.stop();
        brPathModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        flPathModule.setDesiredState(desiredStates[0]);
        frPathModule.setDesiredState(desiredStates[1]);
        blPathModule.setDesiredState(desiredStates[2]);
        brPathModule.setDesiredState(desiredStates[3]);
    }
}

package frc.robot.commands.Autos;

import java.util.List;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Follow extends CommandBase
{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public HolonomicDriveController controller;
    public SwerveModule m_frontLeftModule;
    public SwerveModule m_backLeftModule;
    public SwerveModule m_frontRightModule;
    public SwerveModule m_backRightModule;
    public TrajectoryConfig trajectoryConfig;
    public SwerveDriveKinematics kinematics;
    double forward;
    double sideways;
    double angular;
    ProfiledPIDController turningController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    Trajectory trajectory;
    //private final SwerveDriveKinematics m_kinematics;

    public Follow(DrivetrainSubsystem driveTrain, SwerveDriveKinematics kinematics) {


        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.kKinematics);
        var controller = new HolonomicDriveController(xController, yController, turningController);

        m_drivetrainSubsystem = driveTrain;
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        addRequirements(m_drivetrainSubsystem);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS*-1 / 2.0),
            // Back left
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS*-1 / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS*-1 / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS*-1 / 2.0)
    );

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);
        
    }

    @Override
    public void execute() {
        // Sample the trajectory at 3.4 seconds from the beginning.
        Trajectory.State goal = trajectory.sample(2);

        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
        ChassisSpeeds adjustedSpeeds = controller.calculate(
        m_drivetrainSubsystem.getPose(), goal, Rotation2d.fromDegrees(0));   
        
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);

        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        ChassisSpeeds driveSpeed = kinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState, backRightState);

        double forward = driveSpeed.vxMetersPerSecond;
        double sideways = driveSpeed.vyMetersPerSecond;
        double angular = driveSpeed.omegaRadiansPerSecond;

        m_drivetrainSubsystem.drive(new ChassisSpeeds(forward, sideways, angular));
    }

    @Override
    public boolean isFinished() {
        return forward == 0 || sideways == 0 || angular == 0;
    }

    @Override
    public void end (boolean interrupted)  {
        // Stops the robot and allows the target distance to be calculated again
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
    }
}

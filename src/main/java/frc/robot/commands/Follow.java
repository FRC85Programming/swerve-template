package frc.robot.commands;

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
    public ProfiledPIDController turningController;
    public SwerveModule m_frontLeftModule;
    public SwerveModule m_backLeftModule;
    public SwerveModule m_frontRightModule;
    public SwerveModule m_backRightModule;
    private final SwerveDriveKinematics m_kinematics;

    public Follow(DrivetrainSubsystem driveTrain)
    {
        HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(0, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(0, 0, 0,
                new TrapezoidProfile.Constraints(60, 60)));
        ProfiledPIDController turningController = new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14));
        m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS*-1 / 2.0),
          // Back left
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS*-1 / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS*-1 / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS*-1 / 2.0)
        );
        m_drivetrainSubsystem = driveTrain;
        m_backRightModule = m_drivetrainSubsystem.getBackRight();
        m_backLeftModule = m_drivetrainSubsystem.getBackLeft();
        m_frontLeftModule = m_drivetrainSubsystem.getFrontLeft();
        m_frontRightModule = m_drivetrainSubsystem.getFrontRight();
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute()
    {   
        // Resets wheels so they don't fight each other
        //m_drivetrainSubsystem.zeroWheels();
        // Configures kinematics so the driving is accurate 
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.kDriveKinematics);
        
        // This sets the trajectory points that will be used as a backup if it can not load the original
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //Go to these locations:
            new Translation2d(0, 1),
            new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees /*spin 180*/  (0)),
        trajectoryConfig);

        // Sample the trajectory at 3.4 seconds from the beginning.
        /*Trajectory.State goal = trajectory.sample(3.4);

        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
        ChassisSpeeds adjustedSpeeds = controller.calculate(
            m_drivetrainSubsystem.getPose(), goal, Rotation2d.fromDegrees(0));

        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

            SwerveModuleState frontLeft = moduleStates[0];
            SwerveModuleState frontRight = moduleStates[1];
            SwerveModuleState backLeft = moduleStates[2];
            SwerveModuleState backRight = moduleStates[3];

        frontLeft = SwerveModuleState.optimize(frontLeft, m_drivetrainSubsystem.getState(m_frontLeftModule).angle);
        m_frontLeftModule.set(frontLeft.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond, turningController.calculate(m_drivetrainSubsystem.getTurningPosition(m_frontLeftModule), frontLeft.angle.getRadians()));
        frontRight = SwerveModuleState.optimize(frontRight, m_drivetrainSubsystem.getState(m_frontRightModule).angle);
        m_frontRightModule.set(frontRight.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond, turningController.calculate(m_drivetrainSubsystem.getTurningPosition(m_frontRightModule), frontRight.angle.getRadians()));
        backRight = SwerveModuleState.optimize(backRight, m_drivetrainSubsystem.getState(m_backRightModule).angle);
        m_backRightModule.set(backRight.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond, turningController.calculate(m_drivetrainSubsystem.getTurningPosition(m_backRightModule), backRight.angle.getRadians()));
        backLeft = SwerveModuleState.optimize(backLeft, m_drivetrainSubsystem.getState(m_backLeftModule).angle);
        m_backLeftModule.set(backLeft.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond, turningController.calculate(m_drivetrainSubsystem.getTurningPosition(m_backLeftModule), backLeft.angle.getRadians()));
        */
    
        
        // Setting up trajectory variables
        /*String trajectoryJSON = "ou
        output/" + auto + ".wpilib.json";
        Trajectory temp;\
        //Load command and select backup if needed
        try{
            if(auto.startsWith("PW_")){
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                temp = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            }else{
                temp = PathPlanner.loadPath(auto, Constants.kPhysicalMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
            }
        }catch(Exception e){
            DriverStation.reportWarning("Error loading path:" + auto + ". Loading backup....", e.getStackTrace());
            temp = trajectory;
        }*/
        // Sets up PID to stay on the trajectory
        PIDController xController = new PIDController(Constants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // This is what actually drives the bot. It is run in a SequentialCommandGroup
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            m_drivetrainSubsystem::getPose,
            Constants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);

        new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_drivetrainSubsystem.stop()));
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}

package frc.robot.commands;

import java.time.chrono.HijrahChronology;
import java.util.List;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowTrajectory {
    public HolonomicDriveController controller;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public FollowTrajectory(DrivetrainSubsystem driveTrain) {
        HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(6.28, 3.14)));
        m_drivetrainSubsystem = driveTrain;
    }

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
        Trajectory.State goal = trajectory.sample(3.4);

        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
        ChassisSpeeds adjustedSpeeds = controller.calculate(
            m_drivetrainSubsystem.getPose(), goal, Rotation2d.fromDegrees(0));

        SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

            SwerveModuleState frontLeft = moduleStates[0];
            SwerveModuleState frontRight = moduleStates[1];
            SwerveModuleState backLeft = moduleStates[2];
            SwerveModuleState backRight = moduleStates[3];

        
        
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
        }
    */
        // Sets up PID to stay on the trajectory
        /*PIDController xController = new PIDController(Constants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);*/

        /*// This is what actually drives the bot. It is run in a SequentialCommandGroup
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            m_drivetrainSubsystem::getPose,
            Constants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);

        return new SequentialCommandGroup(
                    new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                    swerveControllerCommand,
                    new InstantCommand(() -> m_drivetrainSubsystem.stop())); */
    }
}

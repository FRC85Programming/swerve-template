// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ExtendAndIntakeCommand;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.IntakeCommand;
import frc.robot.commands.Arm.ManualExtendoCommand;
import frc.robot.commands.Autos.CubeHighBalanceNoMobility;
import frc.robot.commands.Autos.ManualMobility;
import frc.robot.commands.Autos.ScoreAndBalance;
import frc.robot.commands.Autos.ScoreConeMidAndPickupCubeNoVision;
import frc.robot.commands.Autos.ScoreCubeHighAndPickupConeNoVision;
import frc.robot.commands.Autos.ScoreLineup;
import frc.robot.commands.Autos.ScorePickupAndBalance;
import frc.robot.commands.Autos.SpinCubeHighAndMobility;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.BrakeWheelsCommand;
import frc.robot.commands.Chassis.DefaultDriveCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.HalfSpeedCommand;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(this);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);
  private final ExtendoSubsystem m_extendoSubsystem = new ExtendoSubsystem();
  private final VisionTracking vision = new VisionTracking();
  
  HolonomicDriveController controller = new HolonomicDriveController(
      new PIDController(1, 0, 0), new PIDController(1, 0, 0),
      new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));

  private HashMap<String, Command> m_autoCommands;
  public PathPlannerTrajectory usedTrajectory;
  Timer pathTimer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.register();
    m_extendoSubsystem.register();
    m_IntakeSubsystem.register();

    SmartDashboard.putString("BobDashAutoMode", "None");

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_extendoSubsystem.setDefaultCommand(new ManualExtendoCommand(m_extendoSubsystem,
        () -> modifyAxis(-m_operatorController.getLeftY()),
        () -> modifyAxis(-m_operatorController.getRightY()),
        () -> getWristAxis()));

    //This generates a PathPlanner path that will make the robot move diagnally and rotate 45 degrees by the end
    // This needs to be working before we can generate our own path

    PathPlannerTrajectory basicTrajectory = PathPlanner.generatePath(
          new PathConstraints(4, 3), 
          new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
          new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(180)) // position, heading
    );

    PathPlannerTrajectory LeftAndForward = PathPlanner.generatePath(
          new PathConstraints(4, 3), 
          new PathPoint(new Translation2d(100, 0), Rotation2d.fromDegrees(0)), // position, heading
          new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)) // position, heading
    );

    PathPlannerTrajectory TwoPiece = PathPlanner.loadPath("TwoPiece", new PathConstraints(4, 3));

    PathPlannerTrajectory OneMDiag = PathPlanner.loadPath("1MDiag", new PathConstraints(6, 3));

    PathPlannerTrajectory OneMStraight = PathPlanner.loadPath("1MStraight", new PathConstraints(.5, .5));

    PathPlannerTrajectory OneMStraightCopy = PathPlanner.loadPath("1MStraightCopy", new PathConstraints(.05, .5));

    PathPlannerTrajectory Rotate180 = PathPlanner.loadPath("Rotate180", new PathConstraints(4, 3));

    PathPlannerTrajectory Crazy = PathPlanner.loadPath("New New New Path", new PathConstraints(4, 3));

    PathPlannerTrajectory DosCube = PathPlanner.loadPath("DosCube", new PathConstraints(.5, 3));



    // m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.zeroPitchRoll();

    // Configure the button bindings
    configureButtonBindings();

    m_autoCommands = new HashMap<String, Command>();
    // RUN THIS AUTO TO TEST THE PATH
    m_autoCommands.put("Basic Path", 
        followTrajectoryCommand(OneMStraightCopy, true));
    m_autoCommands.put("Bump-MidCone-Pickup-Red", 
      new ScoreConeMidAndPickupCubeNoVision(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem, Alliance.Red));
    m_autoCommands.put("Bump-MidCone-Pickup-Blue", 
      new ScoreConeMidAndPickupCubeNoVision(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem, Alliance.Blue));
    m_autoCommands.put("Bump-MidCone-Pickup-Red-No-Vision", 
      new ScoreConeMidAndPickupCubeNoVision(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem, Alliance.Red));
    m_autoCommands.put("Bump-MidCone-Pickup-Cube-Blue-No-Vision", 
      new ScoreConeMidAndPickupCubeNoVision(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem, Alliance.Blue));
    m_autoCommands.put("Bump-CubeHigh-Pickup-Cone-Blue-NoVision",
      new ScoreCubeHighAndPickupConeNoVision(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem, Alliance.Blue));
    m_autoCommands.put("Bump-CubeHigh-Pickup-Cone-Red-NoVision",
      new ScoreCubeHighAndPickupConeNoVision(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem, Alliance.Red));
    // m_autoCommands.put("Bump-HighCube-Mobility",
    //   new CubeHighAndMobility(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem));
    //   m_autoCommands.put("Bump-MidCube-Mobility",
    //   new CubeHighAndMobility(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem));
    m_autoCommands.put("HighCube-Mobility-Spin",
      new SpinCubeHighAndMobility(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem));
    m_autoCommands.put("MidCube-Mobility-Spin",
      new SpinCubeHighAndMobility(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem));
    m_autoCommands.put("Center-HighCube-Mobility-Balance",
      new ScoreAndBalance(m_drivetrainSubsystem, vision, m_extendoSubsystem, m_IntakeSubsystem, "cube high"));
    m_autoCommands.put("Center-HighCube-Balance",
      new CubeHighBalanceNoMobility(m_drivetrainSubsystem, vision, m_extendoSubsystem, m_IntakeSubsystem, "cube high"));
    m_autoCommands.put("Center-MidCone-Balance",
      new CubeHighBalanceNoMobility(m_drivetrainSubsystem, vision, m_extendoSubsystem, m_IntakeSubsystem, "cone middle"));
    m_autoCommands.put("Mobility-NoScore", 
      new ManualMobility(m_drivetrainSubsystem, vision, m_IntakeSubsystem, this));
    // m_autoCommands.put("Bump-MidCone-Mobility", 
    //   new ConeMidAndMobility(m_drivetrainSubsystem, vision, null, m_extendoSubsystem, m_IntakeSubsystem));
    m_autoCommands.put("ScorePickUpandBalance",
      new ScorePickupAndBalance(m_drivetrainSubsystem, vision, this, m_extendoSubsystem, m_IntakeSubsystem));

    Set<String> autoKeys = m_autoCommands.keySet();
    SmartDashboard.putStringArray("AutoModes", autoKeys.toArray(new String[autoKeys.size()]));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Trigger(m_controller::getBackButton)
        // No requirements because we don't need to interrupt anything
        .onTrue(new ZeroGyroscopeCommand(m_drivetrainSubsystem, 0));
    new Trigger(m_controller::getStartButton)
        .onTrue(new ZeroPitchRollCommand(m_drivetrainSubsystem));
    new Trigger(() -> m_controller.getPOV() == 180)
        .whileTrue(new AutoLevelPIDCommand(m_drivetrainSubsystem));

    // tracks april tag using limelight
    // new Trigger(m_controller::getYButton)
    // .whileTrue(new TrackAprilTagCommand(m_drivetrainSubsystem,
    // m_visionTracking));

    // new Trigger(m_controller::getLeftBumper)
    // .whileTrue(new IntakeCommand(m_IntakeSubsystem, true));

    // new Trigger(m_controller::getRightBumper)
    // .whileTrue(new IntakeCommand(m_IntakeSubsystem, false));

    // intake
    new Trigger(() -> m_controller.getLeftTriggerAxis() != 0)
        .whileTrue(new IntakeCommand(m_IntakeSubsystem, () -> m_controller.getLeftTriggerAxis()));
    // outtake
    new Trigger(() -> m_controller.getRightTriggerAxis() != 0)
        .whileTrue(new IntakeCommand(m_IntakeSubsystem, () -> -m_controller.getRightTriggerAxis()));

    // a button activates brake wheels command
    new Trigger(() -> m_controller.getPOV() == 90)
        .whileTrue(new BrakeWheelsCommand(m_drivetrainSubsystem));

    // // Cuts robot speed in half
    // new Trigger(() -> m_controller.getPOV() == 270)
    //     .whileTrue(new HalfSpeedCommand(m_drivetrainSubsystem));

    // new Trigger(m_operatorController::getBButton)
    // .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 23.0, () -> 69.0, ()
    // -> -60.5));

    new Trigger(() -> m_controller.getXButton() || SmartDashboard.getBoolean("BobDashPositionHold", false))
        .whileTrue(new ExtendCommand(m_extendoSubsystem,
        () -> SmartDashboard.getNumber("DesiredExtendPosition", 0),
        () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
        () -> SmartDashboard.getNumber("DesiredWristPosition", 0)));
        //() -> SmartDashboard.getNumber("DesiredRollerSpeed", 0)));

    new Trigger(m_controller::getLeftBumper)
        .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0, false, false));

    new Trigger(m_controller::getAButton)
        .whileTrue(new ScoreLineup(m_drivetrainSubsystem, vision, this, false));

    new Trigger(m_operatorController::getAButton)
        .whileTrue(new HomeExtendCommand(m_extendoSubsystem));

    // cube pick up position
    // new Trigger(m_controller::getAButton)
    // .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 47.0, () -> 30.0, ()
    // -> -23.0));

    // cone pick up position (Tipped)
    // new Trigger(m_controller::getXButton)
    // .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 52.0, () -> 34.0, ()
    // -> -44.0));

    // // cone pick up position (Upright)
    // new Trigger(m_controller::getYButton)
    // .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 23.0, () -> 69.0, ()
    // -> -60.5));
  }

  public boolean checkOpController() {
    return m_operatorController.getXButton();
  }

  public boolean checkDriveController() {
    return m_operatorController.getXButton();
  }

    public VisionTracking getVision() {
      return vision;
    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    String autoMode = SmartDashboard.getString("BobDashAutoMode", "None");
    if (m_autoCommands.containsKey(autoMode)) {
      return m_autoCommands.get(autoMode);
    } else {
      return new HomeExtendCommand(m_extendoSubsystem);
    }
  }

  public double getLeftY() {
    return -modifyAxis(m_controller.getLeftY());
  }

  public void writeDriveSpeeds() {
    SmartDashboard.putNumber("FL Drive Speed", m_drivetrainSubsystem.getFrontLeft().getDriveMotor().get());
    SmartDashboard.putNumber("FR Drive Speed", m_drivetrainSubsystem.getFrontRight().getDriveMotor().get());
    SmartDashboard.putNumber("BL Drive Speed", m_drivetrainSubsystem.getBackLeft().getDriveMotor().get());
    SmartDashboard.putNumber("BR Drive Speed", m_drivetrainSubsystem.getBackRight().getDriveMotor().get());

  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    usedTrajectory = traj;
    pathTimer.start();
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
              // If we are running our first path of the auto it will set an origin point to where are robot starts at the beggining of the auto
               m_drivetrainSubsystem.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         
         new PPSwerveControllerCommand(
             traj, 
             // Note: the :: as opposed to . when calling a function means that it is refrencing the function, but not running it, as the 
             //PPSwerveControllerCommand will run it in its own code from pathplanner when the followTrajectoryCommand is run.
             m_drivetrainSubsystem::NewGetPose, // Supplies command with the function to get the robot's position
             m_drivetrainSubsystem.getKinematics(), // Wheelbase, tracklength, and other variables
             new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             m_drivetrainSubsystem::setModuleStates, // This function is most likley where broken code would be that causes the robot to not move at all
             false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             m_drivetrainSubsystem // Requires this drive subsystem
         )
     );
  }

  public PathPlannerTrajectory getTraj() {
    return usedTrajectory;
  }

  /*public Rotation2d getAutoFieldRot() {
    State pathState = getTraj().sample(pathTimer.get());
    Rotation2d autoAngle = pathState.();


    return autoAngle;
  }*/

  /**
   * 
   * @param value
   * @param deadband
   * @return
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.00) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * 
   * @param value
   * @return
   */
  private double getWristAxis() {
    if (m_operatorController.getRightBumper()) {
      return 1;
    } else if (modifyAxis(m_operatorController.getRightTriggerAxis()) != 0) {
      return -m_operatorController.getRightTriggerAxis();
    } else {
      return 0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.12);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * 
   * @return the controller object
   */
  public XboxController getController() {
    return m_controller;
  }

  public void checkCalibration() {
    m_drivetrainSubsystem.checkCalibration();
  }

  public void zeroEncoders() {
    m_extendoSubsystem.zeroEncodersIfLimits();
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);
  private final ExtendoSubystem m_ExtendoSubystem = new ExtendoSubystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.register();
    m_ExtendoSubystem.register();
    m_IntakeSubsystem.register();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            ));

    m_ExtendoSubystem.setDefaultCommand(new ManualExtendoCommand(m_ExtendoSubystem, 
            () -> modifyAxis(-m_operatorController.getLeftY()), 
            () -> modifyAxis(-m_operatorController.getRightY())));
          
    m_IntakeSubsystem.setDefaultCommand(new IntakeWristCommand(m_IntakeSubsystem,
            () -> getWristAxis()));

    //m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.zeroPitchRoll();
    
    // Configure the button bindings
    configureButtonBindings();
  }

  public double getX()
  {

    if (m_controller.getPOV() == 90) {
      return 1;
    } else if (m_controller.getPOV() == 270){
      return -1;
    }
    else {
      return m_controller.getLeftX();
    }
  }

  public double getY()
  {

    if (m_controller.getPOV() == 0) {
      return -1;
    } else if (m_controller.getPOV() == 180){
      return 1;
    }
    else {
      return m_controller.getLeftY();
    }
  }

  
 
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Trigger(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
              .onTrue(new ZeroGyroscopeCommand(m_drivetrainSubsystem));
    new Trigger(m_operatorController::getStartButton)
              .onTrue(new ZeroPitchRollCommand(m_drivetrainSubsystem));
    new Trigger(m_controller::getXButton)
              .whileTrue(new AutoLevelPIDCommand(m_drivetrainSubsystem));

    // tracks april tag using limelight
    // new Trigger(m_controller::getYButton)
    //         .whileTrue(new TrackAprilTagCommand(m_drivetrainSubsystem, m_visionTracking));

    // a button activates brake wheels command
    // new Trigger(m_controller::getLeftBumper)
    //         .whileTrue(new BrakeWheelsCommand(m_drivetrainSubsystem));

    // // Cuts robot speed in half 
    // new Trigger(m_controller::getRightBumper)
    //         .whileTrue(new HalfSpeedCommand(m_drivetrainSubsystem));

    // cube pick up position
    new Trigger(m_controller::getBButton)
            .whileTrue(new ExtendCommand(m_ExtendoSubystem, m_IntakeSubsystem, 47.0, 30.0, -23.0));

    // cone pick up position (Tipped)
    new Trigger(m_controller::getAButton)
            .whileTrue(new ExtendCommand(m_ExtendoSubystem, m_IntakeSubsystem, 52.0, 34.0, -44.0));

    // cone pick up position (Upright)
    new Trigger(m_controller::getYButton)
            .whileTrue(new ExtendCommand(m_ExtendoSubystem, m_IntakeSubsystem, 23.0, 69.0, -60.5));

    new Trigger(m_controller::getLeftBumper)
            .whileTrue(new IntakeCommand(m_IntakeSubsystem, true));

    new Trigger(m_controller::getRightBumper)
            .whileTrue(new IntakeCommand(m_IntakeSubsystem, false));    
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Configures kinematics so the driving is accurate 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.kMaxSpeedMetersPerSecond,
      Constants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(Constants.kDriveKinematics);
    
    // This actually makes the trajectory. This will be changed to use pathweaver, but now it has a basic path
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        // Go to these locations:
        new Translation2d(1, 0),
        new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees /*spin 180 */ (180)),
      trajectoryConfig);

    // Sets up PID
    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Makes the command to drive in auto
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
                // Ask Poom what this error means
                new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                // You have to add stop modules for this error. Look at this code and copy and paste: https://github.com/SeanSun6814/FRC0ToAutonomous/tree/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot
                new InstantCommand(() -> m_drivetrainSubsystem.stop()));
  }

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
  private double getWristAxis() {
    if(m_operatorController.getRightBumper()){
      return 0.5;
    } else if (modifyAxis(m_operatorController.getRightTriggerAxis()) != 0){
      return -0.5;
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

  public XboxController getController()
  {
    return m_controller;
  }

  public void checkCalibration() {
    m_drivetrainSubsystem.checkCalibration();
  }
}
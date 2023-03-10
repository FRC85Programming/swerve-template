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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.IntakeCommand;
import frc.robot.commands.Arm.ManualExtendoCommand;
import frc.robot.commands.Autos.BalanceAuto;
import frc.robot.commands.Autos.CS;
import frc.robot.commands.Autos.ManualMobility;
import frc.robot.commands.Autos.ManualOnePlace;
import frc.robot.commands.Autos.ScoreBalanceAuto;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.BrakeWheelsCommand;
import frc.robot.commands.Chassis.DefaultDriveCommand;
import frc.robot.commands.Chassis.HalfSpeedCommand;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;

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
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);
  private final ExtendoSubsystem m_extendoSubsystem = new ExtendoSubsystem();

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

    // m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.zeroPitchRoll();

    // Configure the button bindings
    configureButtonBindings();
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
    new Trigger(() -> m_controller.getPOV() == 270)
        .whileTrue(new HalfSpeedCommand(m_drivetrainSubsystem));

    // new Trigger(m_operatorController::getBButton)
    // .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 23.0, () -> 69.0, ()
    // -> -60.5));

    new Trigger(m_controller::getXButton)
        .whileTrue(new ExtendCommand(m_extendoSubsystem,
            () -> SmartDashboard.getNumber("DesiredExtendPosition", 0),
            () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
            () -> SmartDashboard.getNumber("DesiredWristPosition", 0)));

    new Trigger(m_controller::getYButton)
        .whileTrue(new ExtendCommand(m_extendoSubsystem,
            () -> 0,
            () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
            () -> SmartDashboard.getNumber("DesiredWristPosition", 0)));

    new Trigger(m_controller::getLeftBumper)
        .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0));

    new Trigger(m_operatorController::getAButton)
        .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0));

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    String autoMode = SmartDashboard.getString("BobDashAutoMode", "None");
    if (autoMode.equals("Manual OnePlace")) {
      return new ManualOnePlace(m_drivetrainSubsystem, this, m_extendoSubsystem, m_IntakeSubsystem);
    } else if (autoMode.equals("Balance")) {
      return new BalanceAuto(m_drivetrainSubsystem, m_extendoSubsystem, m_IntakeSubsystem);
    } else if (autoMode.equals("Score and Engage")) {
      return new ScoreBalanceAuto(m_drivetrainSubsystem, m_extendoSubsystem, m_IntakeSubsystem);
    } else if (autoMode.equals("Manual Mobility")) {
      return new ManualMobility(m_drivetrainSubsystem, this);
    } else if (autoMode.equals("CS")) {
      return new CS(m_drivetrainSubsystem, this);
    } else if (autoMode.equals("Normal Follow")) {
      return getAutonomousCommand();
    } else {
      return new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0);
    }
  }

  public Command getAutonomousCommand(/* String auto */) {
    // Resets wheels so they don't fight each other
    // m_drivetrainSubsystem.zeroWheels();
    // Configures kinematics so the driving is accurate
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.kDriveKinematics);

    // This sets the trajectory points that will be used as a backup if it can not
    // load the original
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            // Go to these locations:
            new Translation2d(5, 0),
            new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees /* spin 180 */ (180)),
        trajectoryConfig);

    // Setting up trajectory variables
    /*
     * String trajectoryJSON = "output/" + auto + ".wpilib.json";
     * Trajectory temp;
     * 
     * //Load command and select backup if needed
     * try{
     * if(auto.startsWith("PW_")){
     * Path trajectoryPath =
     * Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
     * temp = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     * }else{
     * temp = PathPlanner.loadPath(auto, Constants.kPhysicalMaxSpeedMetersPerSecond,
     * Constants.kMaxAccelerationMetersPerSecondSquared);
     * }
     * }catch(Exception e){
     * DriverStation.reportWarning("Error loading path:" + auto +
     * ". Loading backup....", e.getStackTrace());
     * temp = trajectory;
     * }
     */
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

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        // You have to add stop modules for this error. Look at this code and copy and
        // paste:
        // https://github.com/SeanSun6814/FRC0ToAutonomous/tree/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot
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

  public XboxController getController() {
    return m_controller;
  }

  public void checkCalibration() {
    m_drivetrainSubsystem.checkCalibration();
  }

}
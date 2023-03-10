// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

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
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  HolonomicDriveController controller = new HolonomicDriveController(
      new PIDController(1, 0, 0), new PIDController(1, 0, 0),
      new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.register();
    m_extendoSubsystem.register();
    m_IntakeSubsystem.register();

    SmartDashboard.putString("BobDashAutoMode", "Normal Follow");

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

    new Trigger(m_controller::getLeftBumper)
        .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0));

    new Trigger(m_operatorController::getAButton)
        .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0));

    // cube pick up position
    // new Trigger(m_controller::getAButton)
    //     .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 47.0, () -> 30.0, () -> -23.0));

    // cone pick up position (Tipped)
    // new Trigger(m_controller::getXButton)
    //     .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 52.0, () -> 34.0, () -> -44.0));

    // // cone pick up position (Upright)
    // new Trigger(m_controller::getYButton)
    //     .whileTrue(new ExtendCommand(m_extendoSubsystem, () -> 23.0, () -> 69.0, () -> -60.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    String autoMode = SmartDashboard.getString("BobDashAutoMode", "Normal Follow");
    if (autoMode.equals("Manual OnePlace")) {
      return new ManualOnePlace(m_drivetrainSubsystem, this, m_extendoSubsystem, m_IntakeSubsystem);
    } else if (autoMode.equals("Balance")) { 
      return new BalanceAuto(m_drivetrainSubsystem, m_extendoSubsystem, m_IntakeSubsystem);
    } else if (autoMode.equals("Score and Engage")) {
      return new ScoreBalanceAuto(m_drivetrainSubsystem, m_extendoSubsystem, m_IntakeSubsystem);
    } else if (autoMode.equals("Manual Mobility")) {
      return new ManualMobility(m_drivetrainSubsystem, m_IntakeSubsystem, this);
    } else if (autoMode.equals("CS")) {
      return new CS(m_drivetrainSubsystem, this);
    } else if (autoMode.equals("Normal Follow")) {
     return new Follow(m_drivetrainSubsystem);
    } else if (autoMode.equals("Score and Pickup")) {
      return new ScoreAndPickup(m_drivetrainSubsystem, this, m_extendoSubsystem, m_IntakeSubsystem);
    } else {
      return new ExtendCommand(m_extendoSubsystem, () -> 0, () -> 0, () -> 0);
    }
  }
  public Command getAutonomousCommand() {
    return null;

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
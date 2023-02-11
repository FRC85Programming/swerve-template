// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ExtendoSubystem;
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
  private final VisionTracking m_visionTracking = new VisionTracking();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);
  private final ExtendoSubystem m_ExtendoSubystem = new ExtendoSubystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    m_ExtendoSubystem.setDefaultCommand(new ManualExtendoCommand(m_ExtendoSubystem, 
            () -> m_operatorController.getLeftY(), 
            () -> m_operatorController.getRightY()));

    // Configure the button bindings
    configureButtonBindings();
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
    //new Trigger(m_controller::getAButtonPressed)
              //.toggleOnTrue(new BrakeWheelsCommand(m_drivetrainSubsystem, true));
    //new Trigger(m_controller::getBButtonPressed)
              //.toggleOnFalse(new BrakeWheelsCommand(m_drivetrainSubsystem, false));

    new Trigger(m_controller::getYButtonPressed)
              .onTrue(new ZeroPitchRollCommand(m_drivetrainSubsystem));
              
    new Trigger(m_controller::getXButton)
              .whileTrue(new AutoLevelCommand(m_drivetrainSubsystem,true));

    // tracks april tag using limelight
    new Trigger(m_controller::getYButton)
            .whileTrue(new TrackAprilTagCommand(m_drivetrainSubsystem, m_visionTracking));

    // a button activates brake wheels command
    new Trigger(m_controller::getAButton)
            .whileTrue(new BrakeWheelsCommand(m_drivetrainSubsystem));

    // Cuts robot speed in half 
    new Trigger(m_controller::getLeftBumper)
            .whileTrue(new HalfSpeedCommand(m_drivetrainSubsystem));

    // Intake roller speed/ button config
    new Trigger(m_controller::getBButton)
            .whileTrue(new IntakeCommand(m_IntakeSubsystem));

    // Extends the extendo arm 
    new Trigger(m_controller::getRightBumper)
            .whileTrue(new ExtendCommand(m_ExtendoSubystem, true));

    // pivots intake arm
    // new Trigger(m_operatorController::getAButton)
    //         .whileTrue(new PivotCommand(m_ExtendoSubystem));

    // new Trigger(m_operatorController::getBButton)
    //         .whileTrue(new PivotCommand(m_ExtendoSubsystem));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
}
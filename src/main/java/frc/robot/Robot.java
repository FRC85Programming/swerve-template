// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //public final DrivetrainSubsystem m_drivetrainSubsystem;
  private RobotContainer m_robotContainer;

  Trajectory trajectory = new Trajectory();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our bCutton bindings, and put our
    // autonomous chooser on the dashboard.
    SmartDashboard.putNumber("Drive and Home Distance", 6.3); //6.35
    SmartDashboard.putNumber("Rotate Speed", -1.4);
    SmartDashboard.putNumber("Rotate Target", 1.4);
    SmartDashboard.putNumber("Intake Start", 0);

    m_robotContainer = new RobotContainer();

    SmartDashboard.putNumber("Roller Speed", 1);
    SmartDashboard.putNumber("AutoLevel Constant", 0.2);
    SmartDashboard.putNumber("AutoLevel Max Speed", .2);
    
    SmartDashboard.putNumber("kp", 1.5);
    SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kd", 0.5);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.checkCalibration();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    //m_autonomousCommand = new CS(m_drivetrainSubsystem, m_robotContainer);
    m_autonomousCommand = m_robotContainer.getAuto();
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand("sda");

    //new Comm_L(m_drivetrainSubsystem, m_robotContainer);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");
    
    inst.startClient4("85"); // Make sure you set this to your team number
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
    
    // NetworkTableEntry TeamEntry = table.getEntry("tx");
    NetworkTableEntry xEntry = table.getEntry("tx");
    NetworkTableEntry yEntry = table.getEntry("ty");
    NetworkTableEntry aEntry = table.getEntry("ta");
    NetworkTableEntry lEntry = table.getEntry("tl");
    NetworkTableEntry vEntry = table.getEntry("tv");
    NetworkTableEntry sEntry = table.getEntry("ts");
    NetworkTableEntry aprilEntry = table.getEntry("tid");
    
    NetworkTableEntry tshortEntry = table.getEntry("tshort");
    NetworkTableEntry tlongEntry = table.getEntry("tlong");
    NetworkTableEntry thorEntry = table.getEntry("thor");
    NetworkTableEntry tvertEntry = table.getEntry("tvert");
    NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
    NetworkTableEntry camtranEntry = table.getEntry("camtran");
    NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
    
    // double tx = xEntry.getDouble(0.0);
    double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    double ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    double ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
    double tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                      // latency.
    double tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
    double ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
    double tid = aprilEntry.getDouble(0.0); // April Tag Number
    
    // double tshort = tshortEntry.getString(); // Sidelength of shortest side of
    // the fitted bounding box (pixels)
    // double tlong = tlong // Sidelength of longest side of the fitted bounding box
    // (pixels)
    // double thor = thor // Horizontal sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double tvert = tvert // Vertical sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double getpipe = getpipe // True active pipeline index of the camera (0 .. 9)
    // double camtran = camtran // Results of a 3D position solution, 6 numbers:
    // Translation (x,y,y) Rotation(pitch,yaw,roll)
    ledModeEntry.setNumber(1);
    //ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
    //ledModeEntry.setNumber(2); // force blink
    //ledModeEntry.setNumber(3); // force on
    
    //System.out.println("X: " + tx);
    //System.out.println("Y: " + ty);
    //System.out.println("A: " + ta);
    //System.out.println("L: " + tl);
    //System.out.println("V: " + tv);
    //System.out.println("S: " + tv);
    
    // post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", tx);
    SmartDashboard.putNumber("Limelight Y", ty);
    SmartDashboard.putNumber("Limelight Area", ta);
    SmartDashboard.putNumber("Limelight Latency", tl);
    SmartDashboard.putNumber("Limelight Valid Target", tv);
    SmartDashboard.putNumber("Limelight Skew", ts);
    SmartDashboard.putNumber("April Tag ID", tid);
    // Limelight Data End

    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d m_field = new Field2d();
  //public final DrivetrainSubsystem m_drivetrainSubsystem;
  private RobotContainer m_robotContainer;

  Trajectory trajectory = new Trajectory();
  NetworkTableInstance inst;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  public void win() {
    // I did this for fun lol
  } 

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our bCutton bindings,
    // and put our
    // autonomous chooser on the dashboard.
    SmartDashboard.putNumber("Drive and Home Distance", 6.3); //6.35
    SmartDashboard.putNumber("Rotate Speed", -1.4);
    SmartDashboard.putNumber("Rotate Target", 1.4);
    SmartDashboard.putNumber("Intake Start", 0);
    SmartDashboard.putData("Field", m_field);

    m_robotContainer = new RobotContainer();
    m_robotContainer.getVision().setLED1(1);
    m_robotContainer.getVision().setLED2(1);

    SmartDashboard.putNumber("Roller Speed", 1);
    SmartDashboard.putNumber("AutoLevel Constant", 0.2);
    SmartDashboard.putNumber("AutoLevel Max Speed", .2);

    SmartDashboard.putNumber("kp", 1.5);
    SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kd", 0.5);
    inst = NetworkTableInstance.getDefault();
    inst.startClient4("85"); // Make sure you set this to your team number
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.checkCalibration();
    m_robotContainer.zeroEncoders();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAuto();

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
    SmartDashboard.putNumber("BobDashMatchTime", DriverStation.getMatchTime());
    // get the default instance of NetworkTables
    // get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");
    
    m_robotContainer.getVision().setLED1(1);
    m_robotContainer.getVision().setLED2(1); 
    
    NetworkTableEntry xEntry = table.getEntry("tx");
    NetworkTableEntry yEntry = table.getEntry("ty");
    NetworkTableEntry aEntry = table.getEntry("ta");
    NetworkTableEntry lEntry = table.getEntry("tl");
    NetworkTableEntry vEntry = table.getEntry("tv");
    NetworkTableEntry sEntry = table.getEntry("ts");
    NetworkTableEntry aprilEntry = table.getEntry("tid");
    NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
    
    double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    double ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    double ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
    double tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                      // latency.
    double tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
    double ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
    double tid = aprilEntry.getDouble(0.0); // April Tag Number
    
    ledModeEntry.setNumber(1);
    
    // post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", tx);
    SmartDashboard.putNumber("Limelight Y", ty);
    SmartDashboard.putNumber("Limelight Area", ta);
    SmartDashboard.putNumber("Limelight Latency", tl);
    SmartDashboard.putNumber("Limelight Valid Target", tv);
    SmartDashboard.putNumber("Limelight Skew", ts);
    SmartDashboard.putNumber("April Tag ID", tid);
    // Limelight Data End

    m_robotContainer.writeDriveSpeeds();

    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}

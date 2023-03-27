// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class comunicates with the network tables and updates the values
 */
public class VisionTracking extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry aprilEntry = table.getEntry("tid");
  NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
  NetworkTableEntry pipelineEntry = table.getEntry("pipeline");

  /** 
   * Creates a new VisionTracking object. 
   */
  public VisionTracking() {
    
  }

  /**
   * Updates the values periodically
   */
  @Override
  public void periodic() {
    // reads values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double validTarget = tv.getDouble(0.0);
    double tid = aprilEntry.getDouble(0.0); // April Tag Number
    double pipeLine = pipelineEntry.getDouble(0.0);

    // posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightValidTarget", validTarget);
    SmartDashboard.putNumber("April Tag ID", tid);
    SmartDashboard.putNumber("Pipeline:", pipeLine);

    // Aims towards target if there is a valid one
    if (validTarget == 1) {
      if (x <= 2 && x >= -2) {
        // put in stop motion?
      } else if (x > 2 && x < 30) {
        // put in turn negative direction
      } else if (x < -2 && x > -30) {
        // put in turn positive direction
      }
    }
  }

  public double getX() {
    return tx.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public double getTag() {
   return aprilEntry.getDouble(0.0);
  }

  public double getLength() {
    return thor.getDouble(0.0);
  }

  public double getHeight() {
    return tvert.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getThor() {
    return thor.getDouble(0.0); // Size of target horizontal
  }

  public void setLED(int ledPower) {
    ledModeEntry.setNumber(ledPower);
  }

  public void setPipeline(int pipeline) {
    pipelineEntry.setNumber(pipeline);
  }
}
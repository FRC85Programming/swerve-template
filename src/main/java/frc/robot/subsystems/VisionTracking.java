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
  NetworkTable table1 = NetworkTableInstance.getDefault().getTable("limelight-one");
  NetworkTableEntry tx1 = table1.getEntry("tx");
  NetworkTableEntry ty1 = table1.getEntry("ty");
  NetworkTableEntry ta1 = table1.getEntry("ta");
  NetworkTableEntry tv1 = table1.getEntry("tv");
  NetworkTableEntry thor1 = table1.getEntry("thor");
  NetworkTableEntry tvert1 = table1.getEntry("tvert");
  NetworkTableEntry aprilEntry1 = table1.getEntry("tid");
  NetworkTableEntry ledModeEntry1 = table1.getEntry("ledMode");
  NetworkTableEntry pipelineEntry1 = table1.getEntry("pipeline");
  NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight-two");
  NetworkTableEntry tx2 = table2.getEntry("tx");
  NetworkTableEntry ty2 = table2.getEntry("ty");
  NetworkTableEntry ta2 = table2.getEntry("ta");
  NetworkTableEntry tv2 = table2.getEntry("tv");
  NetworkTableEntry thor2 = table2.getEntry("thor");
  NetworkTableEntry tvert2 = table2.getEntry("tvert");
  NetworkTableEntry aprilEntry2 = table2.getEntry("tid");
  NetworkTableEntry ledModeEntry2 = table2.getEntry("ledMode");
  NetworkTableEntry pipelineEntry2 = table2.getEntry("pipeline");

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
    double x1 = tx1.getDouble(0.0);
    double y1 = ty1.getDouble(0.0);
    double area1 = ta1.getDouble(0.0);
    double validTarget1 = tv1.getDouble(0.0);
    double tid1 = aprilEntry1.getDouble(0.0); // April Tag Number
    double pipeLine1 = pipelineEntry1.getDouble(0.0);

    double x2 = tx2.getDouble(0.0);
    double y2 = ty2.getDouble(0.0);
    double area2 = ta2.getDouble(0.0);
    double validTarget2 = tv2.getDouble(0.0);
    double tid2 = aprilEntry2.getDouble(0.0); // April Tag Number
    double pipeLine2 = pipelineEntry2.getDouble(0.0);

    // posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX1", x1);
    SmartDashboard.putNumber("LimelightY1", y1);
    SmartDashboard.putNumber("LimelightArea1", area1);
    SmartDashboard.putNumber("LimelightValidTarget1", validTarget1);
    SmartDashboard.putNumber("April Tag ID1", tid1);
    SmartDashboard.putNumber("Pipeline1", pipeLine1);

    SmartDashboard.putNumber("LimelightX2", x2);
    SmartDashboard.putNumber("LimelightY2", y2);
    SmartDashboard.putNumber("LimelightArea2", area2);
    SmartDashboard.putNumber("LimelightValidTarget2", validTarget2);
    SmartDashboard.putNumber("April Tag ID2", tid2);
    SmartDashboard.putNumber("Pipeline2", pipeLine2);
  }

  public double getX1() {
    return tx1.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  }

  public double getArea1() {
    return ta1.getDouble(0.0);
  }

  public double getTag1() {
   return aprilEntry1.getDouble(0.0);
  }

  public double getLength1() {
    return thor1.getDouble(0.0);
  }

  public double getHeight1() {
    return tvert1.getDouble(0.0);
  }

  public double getY1() {
    return ty1.getDouble(0.0);
  }

  public double getThor1() {
    return thor1.getDouble(0.0); // Size of target horizontal
  }

  public void setLED1(int ledPower) {
    ledModeEntry1.setNumber(ledPower);
  }

  public void setPipeline1(int pipeline) {
    pipelineEntry1.setNumber(pipeline);
  }

  public double getX2() {
    return tx2.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  }

  public double getArea2() {
    return ta2.getDouble(0.0);
  }

  public double getTag2() {
   return aprilEntry2.getDouble(0.0);
  }

  public double getLength2() {
    return thor2.getDouble(0.0);
  }

  public double getHeight2() {
    return tvert2.getDouble(0.0);
  }

  public double getY2() {
    return ty2.getDouble(0.0);
  }

  public double getThor2() {
    return thor2.getDouble(0.0); // Size of target horizontal
  }

  public void setLED2(int ledPower) {
    ledModeEntry2.setNumber(ledPower);
  }

  public void setPipeline2(int pipeline) {
    pipelineEntry2.setNumber(pipeline);
  }
}
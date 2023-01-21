// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionTracking extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry aprilEntry = table.getEntry("tid");
  NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
  /** Creates a new VisionTracking. */
  public VisionTracking() {
    
   




  }

  @Override
  public void periodic() {
    //reads values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double validTarget = tv.getDouble(0.0);
    double tid = aprilEntry.getDouble(0.0); // April Tag Number

    //posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightValidTarget", validTarget);
    SmartDashboard.putNumber("April Tag ID", tid);

//Aims towards target if there is a valid one
if (validTarget == 1); {
  
}
  }
  public double getX(){
    return tx.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  }

  public void setLED(int LEDMode){
    ledModeEntry.setNumber(LEDMode);
  }
}

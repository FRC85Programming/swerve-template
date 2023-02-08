package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoLevelPIDCommand extends CommandBase {

    public static double usfullDrivePower = .7 ;
    public DrivetrainSubsystem m_drivetrainSubsystem;

    public AutoLevelPIDCommand(DrivetrainSubsystem drivetrainSubsystem){
        this.m_drivetrainSubsystem = drivetrainSubsystem;        
    }

    @Override
    public void execute()
    {
        m_drivetrainSubsystem.AutoLevelPIDController();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        
    }
}
package frc.robot.commands;

import javax.swing.GroupLayout.SequentialGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;

public class MobilityAuto extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public MobilityAuto(DrivetrainSubsystem driveTrain) {
        // Sets up variables for each subsystem
        m_drivetrainSubsystem = driveTrain;
    }


    @Override
    public void execute() {
        
    }

    @Override
    public void end (boolean interrupted)  {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0, 0));
    }
}
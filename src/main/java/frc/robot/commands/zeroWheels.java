package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

import com.swervedrivespecialties.swervelib.SwerveModule;

public class zeroWheels extends CommandBase {
    private final DrivetrainSubsystem m_DrivetrainSubsystem;



    public zeroWheels(DrivetrainSubsystem m_DrivetrainSubsystem) {

        this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
    }

    @Override
    public void execute() {
        m_DrivetrainSubsystem.getFrontLeft().set(0, 0 - -m_DrivetrainSubsystem.getFrontLeftCalibration());
        m_DrivetrainSubsystem.getBackLeft().set(0, 0 - -m_DrivetrainSubsystem.getBackLeftCalibration());
        m_DrivetrainSubsystem.getFrontRight().set(0, 0 - -m_DrivetrainSubsystem.getFrontRightCalibration());
        m_DrivetrainSubsystem.getBackRight().set(0, 0 - -m_DrivetrainSubsystem.getBackRightCalibration());
        
    }

    @Override
    public void end(boolean interrupted) {
        m_DrivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        
    }
}
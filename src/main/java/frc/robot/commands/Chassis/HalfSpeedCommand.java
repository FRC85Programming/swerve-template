package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HalfSpeedCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public HalfSpeedCommand(DrivetrainSubsystem driveTrain) {
        this.m_drivetrainSubsystem = driveTrain;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.setHalfSpeed(true);
    }

    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_drivetrainSubsystem.setLock(false);
    }
}

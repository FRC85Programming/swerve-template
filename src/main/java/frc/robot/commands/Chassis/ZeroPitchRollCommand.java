package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroPitchRollCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public ZeroPitchRollCommand(DrivetrainSubsystem driveTrain) {
        this.m_drivetrainSubsystem = driveTrain;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroPitchRoll();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

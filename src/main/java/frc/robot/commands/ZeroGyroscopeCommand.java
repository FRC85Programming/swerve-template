package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroscopeCommand extends CommandBase
{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public ZeroGyroscopeCommand(DrivetrainSubsystem driveTrain)
    {
        this.m_drivetrainSubsystem = driveTrain;
    }

    @Override
    public void execute()
    {
        m_drivetrainSubsystem.zeroGyroscope();
    }
}

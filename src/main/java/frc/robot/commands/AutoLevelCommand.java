package frc.robot.commands;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevelCommand extends CommandBase
{

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public AutoLevelCommand(DrivetrainSubsystem drivetrain)
    {
        this.m_drivetrainSubsystem = drivetrain;
    }


    @Override
    public void execute()
    {
       double[] ypr = m_drivetrainSubsystem.GetPitchRoll();
    }
}

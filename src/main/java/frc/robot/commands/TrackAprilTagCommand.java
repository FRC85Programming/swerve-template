package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrackAprilTagCommand extends CommandBase
{
    private boolean trackTag = false;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public TrackAprilTagCommand(DrivetrainSubsystem driveTrain)
    {
        this.m_drivetrainSubsystem = driveTrain;
    }

    @Override
    public void execute()
    {
        trackTag = true;
    }

    public boolean getTrackTag(){
        return trackTag;
    }
}

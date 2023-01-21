package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class TrackAprilTagCommand extends CommandBase
{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public TrackAprilTagCommand(DrivetrainSubsystem driveTrain)
    {
        this.m_drivetrainSubsystem = driveTrain;
    }

    @Override
    public void execute()
    {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get a reference to the subtable called "datatable"
        NetworkTable table = inst.getTable("limelight");
        NetworkTableEntry xEntry = table.getEntry("tx");
        double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, tx));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        
    }
}

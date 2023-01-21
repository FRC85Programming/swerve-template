package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class TrackAprilTagCommand extends CommandBase
{
    double wheelAngle = 0.0;
    boolean wheelReset = true;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    public TrackAprilTagCommand(DrivetrainSubsystem driveTrain)
    {
        this.m_drivetrainSubsystem = driveTrain;
    }

    @Override
    public void execute()
    {   
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("limelight");
        NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
        if (wheelReset == true){
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0));
            ledModeEntry.setNumber(1);
            wheelReset = false;
        }
        // get a reference to the subtable called "datatable"
        NetworkTableEntry xEntry = table.getEntry("tx");
        double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, tx));
        //wheelAngle += 1; 

        }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        wheelAngle = 0;
        wheelReset = true;
    }
}

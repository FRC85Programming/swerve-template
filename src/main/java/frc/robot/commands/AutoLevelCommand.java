package frc.robot.commands;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoLevelCommand extends CommandBase
{

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final boolean m_stationStateOn;
    public AutoLevelCommand(DrivetrainSubsystem drivetrain, boolean stationStateOn)
    {
        this.m_drivetrainSubsystem = drivetrain;
        this.m_stationStateOn = stationStateOn;
    }


    @Override
    public void execute()
    {
    
    //Code to AutoLevel the Robot Forwards and Backwards if the Robot drives onto the Charging Station Facing Forward
       double[] ypr = m_drivetrainSubsystem.GetPitchRoll();

       if (ypr[1] > 10) 
    {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(-.5,0,0));
    }
    else if (ypr[1] < -10) 
    {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(.5,0,0));
    }
    }

    //Code to AutoLevel the Robot Forwards and Backwards if the Robot drives onto the Charging Station Facing The Front Right Module
    


    @Override
    public boolean isFinished()
    {
        double[] ypr = m_drivetrainSubsystem.GetPitchRoll();
        
        
    //Code to stop the Robot from AutoLeveling when Leveled facing forwards
        if (ypr[1] < -5 || ypr[1] > 5) {
            return false;
        }
        else
        {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        
    }
}

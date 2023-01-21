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
       double[] ypr = m_drivetrainSubsystem.GetPitchRoll();
       m_drivetrainSubsystem.setStationState(m_stationStateOn);

       if (ypr[1] > 10) 
    {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(-.5,0,0));
    }
    else if (ypr[1] < -10) 
    {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(.5,0,0));
    }
    }

    @Override
    public boolean isFinished()
    {
        double[] ypr = m_drivetrainSubsystem.GetPitchRoll();
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

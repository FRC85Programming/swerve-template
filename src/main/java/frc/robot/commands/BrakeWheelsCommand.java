package frc.robot.commands;

import javax.swing.text.AbstractDocument.BranchElement;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BrakeWheelsCommand extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final boolean m_brakesOn;
    public BrakeWheelsCommand(DrivetrainSubsystem driveTrain, boolean brakesOn)
    {
        this.m_drivetrainSubsystem = driveTrain;
        this.m_brakesOn = brakesOn;
        }

    @Override
    public void execute()
    {
        m_drivetrainSubsystem.setLock(m_brakesOn);
    }
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TankdriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final XboxController m_Controller;
    public TankdriveCommand (DrivetrainSubsystem driveTrain, XboxController controller)
    {
        this.m_drivetrainSubsystem = driveTrain;
        this.m_Controller = controller;
    }

    @Override
    public void execute(){
        m_drivetrainSubsystem.tankState(m_Controller);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));

    }
}

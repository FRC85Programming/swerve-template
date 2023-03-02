package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoScore extends CommandBase
 {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;


    public AutoScore(DrivetrainSubsystem driveTrain, ExtendoSubsystem extendo, IntakeSubsystem intake)
    {
        m_drivetrainSubsystem = driveTrain;
        m_extendoSubsystem = extendo;
        m_intakeSubsystem = intake;

        addRequirements(m_drivetrainSubsystem, extendo, intake);
    }

    @Override
    public void execute()
    {
        m_extendoSubsystem.Pivot(0.8, 0);
        /*m_intakeSubsystem.setRollerSpeed(-0.8);

        m_intakeSubsystem.StopRollers();*/

    }

    public boolean isFinished() {
        return m_extendoSubsystem.getPivotAngle() >= 40;
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
    }
}


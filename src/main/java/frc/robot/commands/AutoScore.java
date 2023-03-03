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
        if (m_extendoSubsystem.getPivotAngle() <= 80) {
            m_extendoSubsystem.Pivot(0.8, 0);
        } else {
            m_extendoSubsystem.Pivot(0, 0);
        }
        if (m_extendoSubsystem.getExtendPosition() <= 60) {
            m_extendoSubsystem.ExtendTelescope(0.2, 0);
        } else {
            m_extendoSubsystem.ExtendTelescope(0, 0);
        }
        if (m_extendoSubsystem.getIntakeWrist() >= -42) {
            m_extendoSubsystem.Wrist(-0.2, 0);
        } else {
            m_extendoSubsystem.Wrist(0, 0);
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);

        m_intakeSubsystem.StopRollers();*/

    }

    public boolean isFinished() {
        return m_extendoSubsystem.getPivotAngle() >= 73 && m_extendoSubsystem.getExtendPosition() >= 53 && m_extendoSubsystem.getIntakeWrist() <= -30;
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
    }
}


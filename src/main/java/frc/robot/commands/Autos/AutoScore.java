package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoScore extends CommandBase
 {
    private final ExtendoSubsystem m_extendoSubsystem;


    public AutoScore(ExtendoSubsystem extendo, String gamePiece)
    {
        m_extendoSubsystem = extendo;

        addRequirements(extendo);
    }

    @Override
    public void execute()
    {
        if (m_extendoSubsystem.getPivotAngle() <= 60) {
            m_extendoSubsystem.Pivot(0.85, 0);
        } else {
            m_extendoSubsystem.Pivot(0, 0);
        }
        if (m_extendoSubsystem.getExtendPosition() <= 60) {
            m_extendoSubsystem.ExtendTelescope(0.5, 0);
        } else {
            m_extendoSubsystem.ExtendTelescope(0, 0);
        }
        if (m_extendoSubsystem.getIntakeWrist() >= -42) {
            m_extendoSubsystem.Wrist(-0.5, 0);
        } else {
            m_extendoSubsystem.Wrist(0, 0);
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);
        m_intakeSubsystem.StopRollers();*/

    }

    public boolean isFinished() {
        return m_extendoSubsystem.getPivotAngle() >= 73 && m_extendoSubsystem.getExtendPosition() >= 53 && m_extendoSubsystem.getIntakeWrist() <= -35;
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
    }
}
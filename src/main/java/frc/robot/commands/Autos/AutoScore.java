package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class AutoScore extends CommandBase
 {
    private final ExtendoSubsystem m_extendoSubsystem;
    private final String gamePiece;


    public AutoScore(ExtendoSubsystem extendo, String gamePiece)
    {
        m_extendoSubsystem = extendo;
        this.gamePiece = gamePiece;

        addRequirements(extendo);
    }

    @Override
    public void execute()
    {
        if (gamePiece == "cube") {
            if (m_extendoSubsystem.getPivotAngle() <= 80) {
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
        }

        if (gamePiece == "cone") {
            if (m_extendoSubsystem.getPivotAngle() <= 80) {
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
        }

    }

    public boolean isFinished() {
        if (gamePiece == "cube") {
            return m_extendoSubsystem.getPivotAngle() >= 73 && m_extendoSubsystem.getExtendPosition() >= 53 && m_extendoSubsystem.getIntakeWrist() <= -35;
        } else {
            return m_extendoSubsystem.getPivotAngle() >= 73 && m_extendoSubsystem.getExtendPosition() >= 53 && m_extendoSubsystem.getIntakeWrist() <= -35;
        }
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
    }
}


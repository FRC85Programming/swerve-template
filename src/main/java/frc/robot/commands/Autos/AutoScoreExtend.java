package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class AutoScoreExtend extends CommandBase
 {
    private final ExtendoSubsystem m_extendoSubsystem;
    private String gamePiecePosition;


    public AutoScoreExtend(ExtendoSubsystem extendo, String gamePiecePosition)
    {
        m_extendoSubsystem = extendo;
        this.gamePiecePosition = gamePiecePosition;

        addRequirements(extendo);
    }

    @Override
    public void execute()
    {
        if (gamePiecePosition.toLowerCase().equals("cube high")) {
            if (m_extendoSubsystem.getExtendPosition() <= 60) {
                m_extendoSubsystem.ExtendTelescope(1, 0);
            } else {
                m_extendoSubsystem.ExtendTelescope(0, 0);
            }
        } else if (gamePiecePosition.toLowerCase().equals("cube middle")) {
                if (m_extendoSubsystem.getExtendPosition() <= 5) {
                    m_extendoSubsystem.ExtendTelescope(1, 0);
                } else {
                    m_extendoSubsystem.ExtendTelescope(0, 0);
                }
        } else if (gamePiecePosition.toLowerCase().equals("cone middle")) {
            if (m_extendoSubsystem.getExtendPosition() <= 50) {
                m_extendoSubsystem.ExtendTelescope(0.8, 0);
            } else {
                m_extendoSubsystem.ExtendTelescope(0, 0);
            }
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);
        m_intakeSubsystem.StopRollers();*/
    }

    public boolean isFinished() {
        if (gamePiecePosition.toLowerCase().equals("cube high")) {
            return m_extendoSubsystem.getExtendPosition() >= 53;
        }
        if (gamePiecePosition.toLowerCase().equals("cube middle")) {
            return m_extendoSubsystem.getExtendPosition() >= 0;
        }
        if (gamePiecePosition.toLowerCase().equals("cone middle")) {
            return m_extendoSubsystem.getExtendPosition() >= 50;
        } else {
            return true;
        }
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
    }
 }

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class AutoScore extends CommandBase
 {
    private final ExtendoSubsystem m_extendoSubsystem;
    private String gamePiecePosition;


    public AutoScore(ExtendoSubsystem extendo, String gamePiecePosition)
    {
        m_extendoSubsystem = extendo;
        this.gamePiecePosition = gamePiecePosition;

        addRequirements(extendo);
    }

    @Override
    public void execute()
    {
        if (gamePiecePosition.toLowerCase().equals("cube high")) {
            if (m_extendoSubsystem.getPivotAngle() <= 60) {
                m_extendoSubsystem.Pivot(1, 0);
            } else {
                m_extendoSubsystem.Pivot(0, 0);
            }
            if (m_extendoSubsystem.getIntakeWrist() >= -23) {
                m_extendoSubsystem.Wrist(-0.7, 0);
            } else {
                m_extendoSubsystem.Wrist(0,0);
            }
        } else if (gamePiecePosition.toLowerCase().equals("cube middle")) {
            if (m_extendoSubsystem.getPivotAngle() <= 10) {
                m_extendoSubsystem.Pivot(1, 0);
            } else {
                m_extendoSubsystem.Pivot(0, 0);
            }
            if (m_extendoSubsystem.getIntakeWrist() >= -10) {
                m_extendoSubsystem.Wrist(-0.7, 0);
            } else {
                m_extendoSubsystem.Wrist(0, 0);
            }
        } else if (gamePiecePosition.toLowerCase().equals("cone middle")) {
            if (m_extendoSubsystem.getPivotAngle() <= 66) {
                m_extendoSubsystem.Pivot(1, 0);
            } else {
                m_extendoSubsystem.Pivot(0, 0);
            }
            if (m_extendoSubsystem.getIntakeWrist() >= -64) {
                m_extendoSubsystem.Wrist(-1, 0);
            } else {
                m_extendoSubsystem.Wrist(0, 0);
            }
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);
        m_intakeSubsystem.StopRollers();*/
    }

    public boolean isFinished() {                       
        if (gamePiecePosition.toLowerCase().equals("cube high")) {
            return m_extendoSubsystem.getPivotAngle() >= 60 && m_extendoSubsystem.getIntakeWrist() <= -23;
        }
        if (gamePiecePosition.toLowerCase().equals("cube middle")) {
            return m_extendoSubsystem.getPivotAngle() >= 7 && m_extendoSubsystem.getIntakeWrist() <= -7;
        }
        if (gamePiecePosition.toLowerCase().equals("cone middle")) {
            return m_extendoSubsystem.getPivotAngle() >= 66 && m_extendoSubsystem.getIntakeWrist() <= -64;
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
package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
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
            if (m_extendoSubsystem.getPivotAngle() <= 70) {
                m_extendoSubsystem.Pivot(0.85, 0);
            } else {
                m_extendoSubsystem.Pivot(0, 0);
            }
            if (m_extendoSubsystem.getExtendPosition() <= 60) {
                m_extendoSubsystem.ExtendTelescope(0.8, 0);
            } else {
                m_extendoSubsystem.ExtendTelescope(0, 0);
            }
            if (m_extendoSubsystem.getIntakeWrist() >= -42) {
                m_extendoSubsystem.Wrist(-0.5, 0);
            } else {
                m_extendoSubsystem.Wrist(0, 0);
            }
        } else if (gamePiecePosition.toLowerCase().equals("cube middle")) {
            if (m_extendoSubsystem.getPivotAngle() <= 44) {
                m_extendoSubsystem.Pivot(0.85, 0);
            } else {
                m_extendoSubsystem.Pivot(0, 0);
            }
            if (m_extendoSubsystem.getExtendPosition() <= 5) {
                m_extendoSubsystem.ExtendTelescope(0.5, 0);
            } else {
                m_extendoSubsystem.ExtendTelescope(0, 0);
            }
            if (m_extendoSubsystem.getIntakeWrist() >= -33) {
                m_extendoSubsystem.Wrist(-0.5, 0);
            } else {
                m_extendoSubsystem.Wrist(0, 0);
            }
        } else if (gamePiecePosition.toLowerCase().equals("cone middle")) {
            if (m_extendoSubsystem.getPivotAngle() <= 79) {
                m_extendoSubsystem.Pivot(0.85, 0);
            } else {
                m_extendoSubsystem.Pivot(0, 0);
            }
            if (m_extendoSubsystem.getExtendPosition() <= 5) {
                m_extendoSubsystem.ExtendTelescope(0.5, 0);
            } else {
                m_extendoSubsystem.ExtendTelescope(0, 0);
            }
            if (m_extendoSubsystem.getIntakeWrist() >= -36) {
                m_extendoSubsystem.Wrist(-0.5, 0);
            } else {
                m_extendoSubsystem.Wrist(0, 0);
            }
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);
        m_intakeSubsystem.StopRollers();*/
    }

    public boolean isFinished() {
        if (gamePiecePosition.toLowerCase().equals("cube high")) {
            return m_extendoSubsystem.getPivotAngle() >= 65 && m_extendoSubsystem.getExtendPosition() >= 53 && m_extendoSubsystem.getIntakeWrist() <= -35;
        }
        if (gamePiecePosition.toLowerCase().equals("cube mid")) {
            return m_extendoSubsystem.getPivotAngle() >= 39 && m_extendoSubsystem.getExtendPosition() >= 0 && m_extendoSubsystem.getIntakeWrist() <= -38;
        }
        if (gamePiecePosition.toLowerCase().equals("cone mid")) {
            return m_extendoSubsystem.getPivotAngle() >= 74 && m_extendoSubsystem.getExtendPosition() >= 0 && m_extendoSubsystem.getIntakeWrist() <= -31;
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
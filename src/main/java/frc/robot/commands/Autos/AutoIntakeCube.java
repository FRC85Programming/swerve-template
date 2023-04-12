package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCube extends CommandBase
 {
    private final ExtendoSubsystem m_extendoSubsystem;


    public AutoIntakeCube(ExtendoSubsystem extendo, IntakeSubsystem intake)
    {
        m_extendoSubsystem = extendo;

        addRequirements(extendo, intake);
    }

    @Override
    public void execute()
    {
        if (m_extendoSubsystem.getPivotAngle() <= 20) {
            m_extendoSubsystem.Pivot(0.85, 0);
        } else {
            m_extendoSubsystem.Pivot(0, 0);
        }
        if (m_extendoSubsystem.getExtendPosition() <= 75) {
            m_extendoSubsystem.ExtendTelescope(0.25, 0);
        } else {
            m_extendoSubsystem.ExtendTelescope(0, 0);
        }
        if (m_extendoSubsystem.getIntakeWrist() >= -24) {
            m_extendoSubsystem.Wrist(-0.3, 0);
        } else {
            m_extendoSubsystem.Wrist(0, 0);
        }
    }

    public boolean isFinished() {
        return m_extendoSubsystem.getPivotAngle() >= 20 && m_extendoSubsystem.getExtendPosition() >= 75 && m_extendoSubsystem.getIntakeWrist() <= -24;
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
    }
 }



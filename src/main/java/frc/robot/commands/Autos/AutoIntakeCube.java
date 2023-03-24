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
        if (m_extendoSubsystem.getPivotAngle() <= 19) {
            m_extendoSubsystem.Pivot(0.85, 0);
        } else {
            m_extendoSubsystem.Pivot(0, 0);
        }
        if (m_extendoSubsystem.getExtendPosition() <= 51) {
            m_extendoSubsystem.ExtendTelescope(0.25, 0);
        } else {
            m_extendoSubsystem.ExtendTelescope(0, 0);
        }
        if (m_extendoSubsystem.getIntakeWrist() >= -34) {
            m_extendoSubsystem.Wrist(-0.3, 0);
        } else {
            m_extendoSubsystem.Wrist(0, 0);
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);

        m_intakeSubsystem.StopRollers();*/

    }

    public boolean isFinished() {
        return m_extendoSubsystem.getPivotAngle() >= 14 && m_extendoSubsystem.getExtendPosition() >= 47 && m_extendoSubsystem.getIntakeWrist() <= -29;
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
    }
 }



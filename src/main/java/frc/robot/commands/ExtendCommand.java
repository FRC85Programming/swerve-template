package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class ExtendCommand extends CommandBase {
    private final ExtendoSubystem m_ExtendoSubystem;
    private final double m_ExtendPosition;
    private final double m_PivotAngle;
    private final double tolerancePivot = 10;
    private final double toleranceExtend = 10;
    private final double extendSpeed = 0.5;
    private final double pivotSpeed = 0.5;

    public ExtendCommand(ExtendoSubystem extendo, double extendPosition, double pivotAngle) {
        m_ExtendoSubystem = extendo;
        m_ExtendPosition = extendPosition;
        m_PivotAngle = pivotAngle;

        addRequirements(extendo);
    }

    @Override
    public void execute() {
        if (m_ExtendoSubystem.getPivotAngle() > m_PivotAngle - tolerancePivot
                && m_ExtendoSubystem.getPivotAngle() < m_PivotAngle + tolerancePivot) {
            m_ExtendoSubystem.Pivot(0);
            if (m_ExtendoSubystem.getExtendPosition() > m_ExtendPosition - toleranceExtend
                    && m_ExtendoSubystem.getExtendPosition() < m_ExtendPosition + toleranceExtend) {
                m_ExtendoSubystem.ExtendTelescope(0);
            } else if (m_ExtendoSubystem.getExtendPosition() > m_ExtendPosition) {
                m_ExtendoSubystem.ExtendTelescope(extendSpeed);
            } else {
                m_ExtendoSubystem.ExtendTelescope(-extendSpeed);
            }
        } else if (m_ExtendoSubystem.getPivotAngle() > m_PivotAngle) {
            m_ExtendoSubystem.Pivot(-pivotSpeed);
        } else {
            m_ExtendoSubystem.Pivot(pivotSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubystem.ExtendTelescope(0.0);
        m_ExtendoSubystem.Pivot(0.0);
    }
}

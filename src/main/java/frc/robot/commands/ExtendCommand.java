package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendCommand extends CommandBase {
    private final ExtendoSubystem m_ExtendoSubystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double m_ExtendPosition;
    private final double m_PivotAngle;
    private final double tolerancePivot = 1.0;
    private final double toleranceExtend = 1.0;
    private final double toleranceIntake = 1.0;
    private final double extendSpeed = 0.8;
    private final double pivotSpeed = 0.8;
    private final double intakePivotSpeed = 0.8;
    private final double m_intakeWrist;

    public ExtendCommand(ExtendoSubystem extendo, IntakeSubsystem intakeSubsystem, double extendPosition,
            double pivotAngle, double intakeWrist) {
        m_IntakeSubsystem = intakeSubsystem;
        m_ExtendoSubystem = extendo;
        m_ExtendPosition = extendPosition;
        m_PivotAngle = pivotAngle;
        m_intakeWrist = intakeWrist;

        addRequirements(extendo);
    }

    @Override
    public void execute() {
        if (m_ExtendoSubystem.getPivotAngle() > m_PivotAngle - tolerancePivot
                && m_ExtendoSubystem.getPivotAngle() < m_PivotAngle + tolerancePivot) {
            m_ExtendoSubystem.Pivot(0.0, 0.0);
            if (m_ExtendoSubystem.getExtendPosition() > m_ExtendPosition - toleranceExtend
                    && m_ExtendoSubystem.getExtendPosition() < m_ExtendPosition + toleranceExtend) {
                m_ExtendoSubystem.ExtendTelescope(0.0, 0.0);
                if (m_IntakeSubsystem.getIntakeWrist() > m_intakeWrist - toleranceIntake
                        && m_IntakeSubsystem.getIntakeWrist() < m_intakeWrist + toleranceIntake) {
                    m_IntakeSubsystem.Pivot(0.0, 0.0);
                } else if (m_IntakeSubsystem.getIntakeWrist() > m_intakeWrist) {
                    m_IntakeSubsystem.Pivot(-intakePivotSpeed, 0.0);
                } else {
                    m_IntakeSubsystem.Pivot(intakePivotSpeed, 0.0);
                }
            } else if (m_ExtendoSubystem.getExtendPosition() > m_ExtendPosition) {
                m_ExtendoSubystem.ExtendTelescope(-extendSpeed, 0.0);
            } else {
                m_ExtendoSubystem.ExtendTelescope(extendSpeed, 0.0);
            }
        } else if (m_ExtendoSubystem.getPivotAngle() > m_PivotAngle) {
            m_ExtendoSubystem.Pivot(-pivotSpeed, 0.0);
        } else {
            m_ExtendoSubystem.Pivot(pivotSpeed, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubystem.ExtendTelescope(0.0, 0.0);
        m_ExtendoSubystem.Pivot(0.0, 0.0);
        m_IntakeSubsystem.Pivot(0.0, 0.0);
    }
}

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendCommand extends CommandBase {
    private final ExtendoSubystem m_ExtendoSubystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final DoubleSupplier m_ExtendPosition;
    private final DoubleSupplier m_PivotAngle;
    private final DoubleSupplier m_intakeWrist;
    private final double tolerancePivot = 1;
    private final double toleranceExtend = 1;
    private final double toleranceIntake = 1;
    private final double extendSpeed = 0.8;
    private final double pivotSpeed = 0.8;
    private final double intakePivotSpeed = 0.8;

    public ExtendCommand(ExtendoSubystem extendo, IntakeSubsystem intakeSubsystem, DoubleSupplier extendPosition,
            DoubleSupplier pivotAngle, DoubleSupplier intakeWrist) {
        m_IntakeSubsystem = intakeSubsystem;
        m_ExtendoSubystem = extendo;
        m_ExtendPosition = extendPosition;
        m_PivotAngle = pivotAngle;
        m_intakeWrist = intakeWrist;

        addRequirements(extendo);
    }

    @Override
    public void execute() {

        double pivotAngle = m_PivotAngle.getAsDouble();
        double intakeWrist = m_intakeWrist.getAsDouble();
        double extendPosition = m_ExtendPosition.getAsDouble();

        if (m_ExtendoSubystem.getPivotAngle() > pivotAngle - tolerancePivot
                && m_ExtendoSubystem.getPivotAngle() < pivotAngle + tolerancePivot) {
            m_ExtendoSubystem.Pivot(0.0, 0.0);
        } else if (m_ExtendoSubystem.getPivotAngle() > pivotAngle) {
            m_ExtendoSubystem.Pivot(-pivotSpeed, 0.0);
        } else {
            m_ExtendoSubystem.Pivot(pivotSpeed, 0.0);
        }

        if (m_ExtendoSubystem.getExtendPosition() > extendPosition - toleranceExtend
                && m_ExtendoSubystem.getExtendPosition() < extendPosition + toleranceExtend) {
            m_ExtendoSubystem.ExtendTelescope(0.0, 0.0);
        } else if (m_ExtendoSubystem.getExtendPosition() > extendPosition) {
            m_ExtendoSubystem.ExtendTelescope(-extendSpeed, 0.0);
        } else {
            m_ExtendoSubystem.ExtendTelescope(extendSpeed, 0.0);
        }

        if (m_IntakeSubsystem.getIntakeWrist() > intakeWrist - toleranceIntake
                && m_IntakeSubsystem.getIntakeWrist() < intakeWrist + toleranceIntake) {
            m_IntakeSubsystem.Pivot(0.0, 0.0);
        } else if (m_IntakeSubsystem.getIntakeWrist() > intakeWrist) {
            m_IntakeSubsystem.Pivot(-intakePivotSpeed, 0.0);
        } else {
            m_IntakeSubsystem.Pivot(intakePivotSpeed, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubystem.ExtendTelescope(0.0, 0.0);
        m_ExtendoSubystem.Pivot(0.0, 0.0);
        m_IntakeSubsystem.Pivot(0.0, 0.0);
    }
}

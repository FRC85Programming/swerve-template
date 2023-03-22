package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class ExtendCommand extends CommandBase {
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final DoubleSupplier m_ExtendPosition;
    private final DoubleSupplier m_PivotAngle;
    private final DoubleSupplier m_IntakeWrist;
    private final double tolerancePivot = 1;
    private final double tolerancePivotSlow = 2;
    private final double toleranceExtend = 1;
    private final double toleranceWrist = 0.6;
    private final double toleranceWristSlow = 4;
    private final double extendSpeed = 0.9;
    private final double pivotFastSpeed = 0.8;
    private final double intakePivotSpeed = 0.8;
    private final double wristSlowSpeed = 0.25;
    private final double pivotSlowSpeed = 0.4;
    private final boolean m_enableZeroing = false;

    public ExtendCommand(ExtendoSubsystem extendo, DoubleSupplier extendPosition,
            DoubleSupplier pivotAngle, DoubleSupplier intakeWrist) {
        this(extendo, extendPosition, pivotAngle, intakeWrist, false);
    }

    public ExtendCommand(ExtendoSubsystem extendo, DoubleSupplier extendPosition,
            DoubleSupplier pivotAngle, DoubleSupplier intakeWrist, boolean enableZeroing) {
        m_ExtendoSubsystem = extendo;
        m_ExtendPosition = extendPosition;
        m_PivotAngle = pivotAngle;
        m_IntakeWrist = intakeWrist;

        addRequirements(extendo);
    }

    @Override
    public void execute() {

        double pivotAngle = m_PivotAngle.getAsDouble();
        double intakeWrist = m_IntakeWrist.getAsDouble();
        double extendPosition = m_ExtendPosition.getAsDouble();

        double pivotSpeed;
        if (m_ExtendoSubsystem.getPivotAngle() > pivotAngle - tolerancePivot
                && m_ExtendoSubsystem.getPivotAngle() < pivotAngle + tolerancePivot) {
            pivotSpeed = 0;
        } else if (m_ExtendoSubsystem.getPivotAngle() > pivotAngle - tolerancePivotSlow
                && m_ExtendoSubsystem.getPivotAngle() < pivotAngle + tolerancePivotSlow) {
            pivotSpeed = pivotSlowSpeed;
        } else {
            pivotSpeed = pivotFastSpeed;
        }

        if (m_ExtendoSubsystem.getPivotAngle() > pivotAngle) {
            m_ExtendoSubsystem.Pivot(-pivotSpeed, 0.0, m_enableZeroing);
        } else {
            m_ExtendoSubsystem.Pivot(pivotSpeed, 0.0, m_enableZeroing);
        }

        if (m_ExtendoSubsystem.getExtendPosition() > extendPosition - toleranceExtend
                && m_ExtendoSubsystem.getExtendPosition() < extendPosition + toleranceExtend) {
            m_ExtendoSubsystem.ExtendTelescope(0.0, 0.0);
        } else if (m_ExtendoSubsystem.getExtendPosition() > extendPosition) {
            m_ExtendoSubsystem.ExtendTelescope(-extendSpeed, 0.0, m_enableZeroing);
        } else {
            m_ExtendoSubsystem.ExtendTelescope(extendSpeed, 0.0, m_enableZeroing);
        }

        double wristSpeed;
        if (m_ExtendoSubsystem.getIntakeWrist() > intakeWrist - toleranceWrist
                && m_ExtendoSubsystem.getIntakeWrist() < intakeWrist + toleranceWrist) {
            wristSpeed = 0;
        } else if (m_ExtendoSubsystem.getIntakeWrist() > intakeWrist - toleranceWristSlow
                && m_ExtendoSubsystem.getIntakeWrist() < intakeWrist + toleranceWristSlow) {
            wristSpeed = wristSlowSpeed;
        } else {
            wristSpeed = intakePivotSpeed;
        }

        if (m_ExtendoSubsystem.getIntakeWrist() > intakeWrist) {
            m_ExtendoSubsystem.Wrist(-wristSpeed, 0.0, m_enableZeroing);
        } else {
            m_ExtendoSubsystem.Wrist(wristSpeed, 0.0, m_enableZeroing);
        }
    }

    @Override
    public boolean isFinished() {
        double pivotAngle = m_PivotAngle.getAsDouble();
        double intakeWrist = m_IntakeWrist.getAsDouble();
        double extendPosition = m_ExtendPosition.getAsDouble();

        return m_ExtendoSubsystem.getIntakeWrist() > intakeWrist - toleranceWrist
                && m_ExtendoSubsystem.getIntakeWrist() < intakeWrist + toleranceWrist
                && m_ExtendoSubsystem.getExtendPosition() > extendPosition - toleranceExtend
                && m_ExtendoSubsystem.getExtendPosition() < extendPosition + toleranceExtend
                && m_ExtendoSubsystem.getPivotAngle() > pivotAngle - tolerancePivot
                && m_ExtendoSubsystem.getPivotAngle() < pivotAngle + tolerancePivot;
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubsystem.ExtendTelescope(0.0, 0.0);
        m_ExtendoSubsystem.Pivot(0.0, 0.0);
        m_ExtendoSubsystem.Wrist(0.0, 0.0);
    }
}

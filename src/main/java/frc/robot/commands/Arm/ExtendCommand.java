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
    private final double tolerancePivotSlow = 4;
    private final double toleranceExtend = 1;
    private final double toleranceExtendSlow = 12;
    private final double toleranceWrist = 1;
    private final double toleranceWristSlow = 4;
    private final double extendSpeedFast = 0.9;
    private final double extendSpeedSlow = 0.5;
    private final double pivotFastSpeed = 0.8;
    private final double intakePivotSpeed = 0.8;
    private final double wristSlowSpeed = 0.25;
    private final double pivotSlowSpeed = 0.4;
    private boolean m_enableZeroing = false;
    private boolean m_endCommandWhenPositionMet = true;

    public ExtendCommand(ExtendoSubsystem extendo, DoubleSupplier extendPosition,
            DoubleSupplier pivotAngle, DoubleSupplier intakeWrist) {
        this(extendo, extendPosition, pivotAngle, intakeWrist, false, true);
    }

    public ExtendCommand(ExtendoSubsystem extendo, DoubleSupplier extendPosition,
            DoubleSupplier pivotAngle, DoubleSupplier intakeWrist, boolean enableZeroing,
            boolean endCommandWhenPositionMet) {
        m_ExtendoSubsystem = extendo;
        m_ExtendPosition = extendPosition;
        m_PivotAngle = pivotAngle;
        m_IntakeWrist = intakeWrist;
        m_enableZeroing = enableZeroing;
        m_endCommandWhenPositionMet = endCommandWhenPositionMet;

        addRequirements(extendo);
    }

    @Override
    public void execute() {

        double pivotAngle = m_PivotAngle.getAsDouble();
        double intakeWrist = m_IntakeWrist.getAsDouble();

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

        if (m_ExtendPosition != null) {
            double extendPosition = m_ExtendPosition.getAsDouble();
            double extendSpeed;
            if (m_ExtendoSubsystem.getExtendPosition() > extendPosition - toleranceExtend
                    && m_ExtendoSubsystem.getExtendPosition() < extendPosition + toleranceExtend) {
                extendSpeed = 0;
            } else if (m_ExtendoSubsystem.getExtendPosition() > extendPosition - toleranceExtendSlow
                    && m_ExtendoSubsystem.getExtendPosition() < extendPosition + toleranceExtendSlow) {
                extendSpeed = extendSpeedSlow;
            } else {
                extendSpeed = extendSpeedFast;
            }

            if (m_ExtendoSubsystem.getExtendPosition() > extendPosition) {
                m_ExtendoSubsystem.ExtendTelescope(-extendSpeed, 0.0, m_enableZeroing);
            } else {
                m_ExtendoSubsystem.ExtendTelescope(extendSpeed, 0.0, m_enableZeroing);
            }
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
        double intakeWrist = Math.abs(m_IntakeWrist.getAsDouble());
        boolean extendFinsished = false;
        if (m_ExtendPosition != null) {
            double extendPosition = m_ExtendPosition.getAsDouble();

            if (extendPosition > ExtendoSubsystem.maxExtend) {
                extendPosition = ExtendoSubsystem.maxExtend;
            }
            extendFinsished = m_ExtendoSubsystem.getExtendPosition() > extendPosition - toleranceExtend
                    && m_ExtendoSubsystem.getExtendPosition() < extendPosition + toleranceExtend;
        } else {
            extendFinsished = true;
        }

        if (pivotAngle > ExtendoSubsystem.maxPivot) {
            pivotAngle = ExtendoSubsystem.maxPivot;
        }

        double wristMax = Math.abs(ExtendoSubsystem.maxWrist);
        if (intakeWrist > wristMax) {
            intakeWrist = wristMax;
        }

        double currentWrist = Math.abs(m_ExtendoSubsystem.getIntakeWrist());
        if (currentWrist > wristMax) {
            currentWrist = wristMax;
        }

        return m_endCommandWhenPositionMet
                && currentWrist > intakeWrist - toleranceWrist
                && currentWrist < intakeWrist + toleranceWrist
                && extendFinsished
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

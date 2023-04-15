package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class ExtendCommand extends CommandBase {
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final DoubleSupplier m_ExtendPosition;
    private final DoubleSupplier m_PivotAngle;
    private final DoubleSupplier m_IntakeWrist;
    private final double tolerancePivot = 1;

    private final double tolerancePivotSlow = 10;
    private final double toleranceExtend = 1;
    private final double toleranceExtendSlow = 12;
    private final double toleranceWrist = 1;
    private final double toleranceWristSlow = 4;
    private final double extendSpeedFast = 1;
    private final double extendSpeedSlow = 0.2;
    private final double pivotFastSpeed = 1;
    private final double intakePivotSpeed = 1;
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

        double desiredPivotAngle = m_PivotAngle.getAsDouble();
        double desiredIntakeWrist = m_IntakeWrist.getAsDouble();

        double currentPivot = m_ExtendoSubsystem.getPivotAngle();
        double currentWrist = m_ExtendoSubsystem.getIntakeWrist();

        boolean atDesiredPivot = currentPivot > desiredPivotAngle - tolerancePivotSlow && currentPivot < desiredPivotAngle + tolerancePivotSlow;
        boolean atDesiredWrist = currentWrist > desiredIntakeWrist - toleranceWristSlow && currentWrist < desiredIntakeWrist + toleranceWristSlow;

        double pivotSpeed;
        if (m_ExtendoSubsystem.getPivotAngle() > desiredPivotAngle - tolerancePivot
                && m_ExtendoSubsystem.getPivotAngle() < desiredPivotAngle + tolerancePivot) {
            pivotSpeed = 0;
        } else if (m_ExtendoSubsystem.getPivotAngle() > desiredPivotAngle - tolerancePivotSlow
                && m_ExtendoSubsystem.getPivotAngle() < desiredPivotAngle + tolerancePivotSlow) {
            pivotSpeed = pivotSlowSpeed;
        } else {
            pivotSpeed = pivotFastSpeed;
        }

        if (m_ExtendoSubsystem.getPivotAngle() > desiredPivotAngle) {
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
            } else if (atDesiredWrist && atDesiredPivot){
                m_ExtendoSubsystem.ExtendTelescope(extendSpeed, 0.0, m_enableZeroing);
            }
        } else {
            m_ExtendoSubsystem.ExtendTelescope(0, 0);
        }

        double wristSpeed;
        if (m_ExtendoSubsystem.getIntakeWrist() > desiredIntakeWrist - toleranceWrist
                && m_ExtendoSubsystem.getIntakeWrist() < desiredIntakeWrist + toleranceWrist) {
            wristSpeed = 0;
        } else if (m_ExtendoSubsystem.getIntakeWrist() > desiredIntakeWrist - toleranceWristSlow
                && m_ExtendoSubsystem.getIntakeWrist() < desiredIntakeWrist + toleranceWristSlow) {
            wristSpeed = wristSlowSpeed;
        } else {
            wristSpeed = intakePivotSpeed;
        }

        if (m_ExtendoSubsystem.getIntakeWrist() > desiredIntakeWrist) {
            m_ExtendoSubsystem.Wrist(-wristSpeed, 0.0, m_enableZeroing);
        } else {
            m_ExtendoSubsystem.Wrist(wristSpeed, 0.0, m_enableZeroing);
        }
    }

    @Override
    public boolean isFinished() {
        double desiredPivotAngle = m_PivotAngle.getAsDouble();
        double desiredIntakeWrist = Math.abs(m_IntakeWrist.getAsDouble());
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

        if (desiredPivotAngle > ExtendoSubsystem.maxPivot) {
            desiredPivotAngle = ExtendoSubsystem.maxPivot;
        }

        double wristMax = Math.abs(ExtendoSubsystem.maxWrist);
        if (desiredIntakeWrist > wristMax) {
            desiredIntakeWrist = wristMax;
        }

        double currentWrist = Math.abs(m_ExtendoSubsystem.getIntakeWrist());
        if (currentWrist > wristMax) {
            currentWrist = wristMax;
        }

        return m_endCommandWhenPositionMet
                && currentWrist > desiredIntakeWrist - toleranceWrist
                && currentWrist < desiredIntakeWrist + toleranceWrist
                && extendFinsished
                && m_ExtendoSubsystem.getPivotAngle() > desiredPivotAngle - tolerancePivot
                && m_ExtendoSubsystem.getPivotAngle() < desiredPivotAngle + tolerancePivot;
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubsystem.ExtendTelescope(0.0, 0.0);
        m_ExtendoSubsystem.Pivot(0.0, 0.0);
        m_ExtendoSubsystem.Wrist(0.0, 0.0);
    }
}

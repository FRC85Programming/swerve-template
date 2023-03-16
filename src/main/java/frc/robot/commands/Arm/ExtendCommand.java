package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendCommand extends CommandBase {
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final DoubleSupplier m_ExtendPosition;
    private final DoubleSupplier m_PivotAngle;
    private final DoubleSupplier m_intakeWrist;
    private final double tolerancePivot = 1;
    private final double toleranceExtend = 1;
    private final double toleranceIntake = 1;
    private final double extendSpeed = 0.8;
    private final double pivotSpeed = 0.8;
    private final double intakePivotSpeed = 0.8;

    public ExtendCommand(ExtendoSubsystem extendo, DoubleSupplier extendPosition,
            DoubleSupplier pivotAngle, DoubleSupplier intakeWrist) {
        m_ExtendoSubsystem = extendo;
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

        if (m_ExtendoSubsystem.getPivotAngle() > pivotAngle - tolerancePivot
                && m_ExtendoSubsystem.getPivotAngle() < pivotAngle + tolerancePivot) {
            m_ExtendoSubsystem.Pivot(0.0, 0.0);
        } else if (m_ExtendoSubsystem.getPivotAngle() > pivotAngle) {
            m_ExtendoSubsystem.Pivot(-pivotSpeed, 0.0);
        } else {
            m_ExtendoSubsystem.Pivot(pivotSpeed, 0.0);
        }

        if (m_ExtendoSubsystem.getExtendPosition() > extendPosition - toleranceExtend
                && m_ExtendoSubsystem.getExtendPosition() < extendPosition + toleranceExtend) {
            m_ExtendoSubsystem.ExtendTelescope(0.0, 0.0);
        } else if (m_ExtendoSubsystem.getExtendPosition() > extendPosition) {
            m_ExtendoSubsystem.ExtendTelescope(-extendSpeed, 0.0);
        } else {
            m_ExtendoSubsystem.ExtendTelescope(extendSpeed, 0.0);
        }

        if (m_ExtendoSubsystem.getIntakeWrist() > intakeWrist - toleranceIntake
                && m_ExtendoSubsystem.getIntakeWrist() < intakeWrist + toleranceIntake) {
            m_ExtendoSubsystem.Wrist(0.0, 0.0);
        } else if (m_ExtendoSubsystem.getIntakeWrist() > intakeWrist) {
            m_ExtendoSubsystem.Wrist(-intakePivotSpeed, 0.0);
        } else {
            m_ExtendoSubsystem.Wrist(intakePivotSpeed, 0.0);
        }
    }

    @Override
    public boolean isFinished(){
        double pivotAngle = m_PivotAngle.getAsDouble();
        double intakeWrist = m_intakeWrist.getAsDouble();
        double extendPosition = m_ExtendPosition.getAsDouble();

        return m_ExtendoSubsystem.getIntakeWrist() > intakeWrist - toleranceIntake
                && m_ExtendoSubsystem.getIntakeWrist() < intakeWrist + toleranceIntake 
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

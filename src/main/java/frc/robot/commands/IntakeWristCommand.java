package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeWristCommand extends CommandBase{
    private final IntakeSubsystem m_intakeSubsystem;
    private final DoubleSupplier m_pivotSpeedSupplier;
    // private final DoubleSupplier m_pivotDownSpeedSupplier;
    public IntakeWristCommand(IntakeSubsystem m_intakeSubsystem, DoubleSupplier m_pivotSpeedSupplier){
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_pivotSpeedSupplier = m_pivotSpeedSupplier;

        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute() {
            m_intakeSubsystem.Pivot(m_pivotSpeedSupplier.getAsDouble(), 0);
        


        SmartDashboard.putNumber("intake wrist speed", m_pivotSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.Pivot(0, 0);
    }
}

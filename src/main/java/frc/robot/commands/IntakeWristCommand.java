package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class IntakeWristCommand extends CommandBase{
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final DoubleSupplier m_pivotSpeedSupplier;
    // private final DoubleSupplier m_pivotDownSpeedSupplier;
    public IntakeWristCommand(ExtendoSubsystem m_extendoSubsystem, DoubleSupplier m_pivotSpeedSupplier){
        this.m_pivotSpeedSupplier = m_pivotSpeedSupplier;
        this.m_ExtendoSubsystem = m_extendoSubsystem;

        addRequirements(m_extendoSubsystem);
    }  

    @Override
    public void execute() {
            m_ExtendoSubsystem.Wrist(m_pivotSpeedSupplier.getAsDouble(), 0);
        
        SmartDashboard.putNumber("intake wrist speed", m_pivotSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubsystem.Wrist(0, 0);
    }
}

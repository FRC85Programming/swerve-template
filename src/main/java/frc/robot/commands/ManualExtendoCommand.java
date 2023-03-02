package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class ManualExtendoCommand extends CommandBase {
    private final ExtendoSubsystem m_ExtendoSubystem;
    private final DoubleSupplier m_pivotSpeedSupplier;
    private final DoubleSupplier m_extendSpeedSupplier;
    
    public ManualExtendoCommand(ExtendoSubsystem m_ExtendoSubystem, DoubleSupplier pivotSpeedSupplier, DoubleSupplier extendSpeedSupplier){
        this.m_ExtendoSubystem = m_ExtendoSubystem;
        this.m_pivotSpeedSupplier = pivotSpeedSupplier;
        this.m_extendSpeedSupplier = extendSpeedSupplier;

        addRequirements(m_ExtendoSubystem);
    }

    @Override
    public void execute(){
        m_ExtendoSubystem.Pivot(m_pivotSpeedSupplier.getAsDouble(), 0);
        m_ExtendoSubystem.ExtendTelescope(m_extendSpeedSupplier.getAsDouble(), 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubystem.Pivot(0, 0);
        m_ExtendoSubystem.ExtendTelescope(0, 0);
    }
}

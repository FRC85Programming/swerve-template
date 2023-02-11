package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class ManualExtendoCommand extends CommandBase {
    private final ExtendoSubystem m_ExtendoSubystem;
    private final DoubleSupplier m_pivotSpeedSupplier;
    private final DoubleSupplier m_extendSpeedSupplier;
    
    public ManualExtendoCommand(ExtendoSubystem m_ExtendoSubystem, DoubleSupplier pivotSpeedSupplier, DoubleSupplier extendSpeedSupplier){
        this.m_ExtendoSubystem = m_ExtendoSubystem;
        this.m_pivotSpeedSupplier = pivotSpeedSupplier;
        this.m_extendSpeedSupplier = extendSpeedSupplier;
    }

    @Override
    public void execute(){
        m_ExtendoSubystem.Pivot(m_pivotSpeedSupplier.getAsDouble());
        m_ExtendoSubystem.ExtendTelescope(m_extendSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubystem.Pivot(0);
        m_ExtendoSubystem.ExtendTelescope(0);
    }
}

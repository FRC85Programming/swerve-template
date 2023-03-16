package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class ManualExtendoCommand extends CommandBase {
    private final ExtendoSubsystem m_ExtendoSubsystem;
    private final DoubleSupplier m_pivotSpeedSupplier;
    private final DoubleSupplier m_extendSpeedSupplier;
    private final DoubleSupplier m_wristSpeedSupplier;
    
    public ManualExtendoCommand(ExtendoSubsystem m_ExtendoSubystem, DoubleSupplier pivotSpeedSupplier, DoubleSupplier extendSpeedSupplier, DoubleSupplier wristSpeedSupplier){
        this.m_ExtendoSubsystem = m_ExtendoSubystem;
        this.m_pivotSpeedSupplier = pivotSpeedSupplier;
        this.m_extendSpeedSupplier = extendSpeedSupplier;
        m_wristSpeedSupplier = wristSpeedSupplier;

        addRequirements(m_ExtendoSubystem);
    }

    @Override
    public void execute(){
        m_ExtendoSubsystem.Pivot(m_pivotSpeedSupplier.getAsDouble(), 0);
        m_ExtendoSubsystem.ExtendTelescope(m_extendSpeedSupplier.getAsDouble(), 0);
        m_ExtendoSubsystem.Wrist(m_wristSpeedSupplier.getAsDouble(), 0);
        
        SmartDashboard.putNumber("intake wrist speed", m_wristSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubsystem.Pivot(0, 0);
        m_ExtendoSubsystem.ExtendTelescope(0, 0);
    }
}

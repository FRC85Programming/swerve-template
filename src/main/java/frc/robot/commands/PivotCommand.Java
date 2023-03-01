package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class PivotCommand extends CommandBase{
    private final ExtendoSubystem m_ExtendoSubsystem;

    public PivotCommand(ExtendoSubystem pivot){
        this.m_ExtendoSubsystem = pivot;
    }

    @Override
    public void execute() {
        // m_ExtendoSubsystem.PivotArmTelescope(0.1);
        m_ExtendoSubsystem.Pivot(0.1, 0);
    }   

    @Override
    public void end(boolean interrupted) {
        m_ExtendoSubsystem.Pivot(0.0, 0);
    }
}
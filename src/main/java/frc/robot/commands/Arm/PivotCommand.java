package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubsystem;

public class PivotCommand extends CommandBase{
    private final ExtendoSubsystem m_ExtendoSubsystem;

    public PivotCommand(ExtendoSubsystem pivot){
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
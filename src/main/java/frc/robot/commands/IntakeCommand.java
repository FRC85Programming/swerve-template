package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem m_intake;

    public IntakeCommand(IntakeSubsystem m_intake){
        this.m_intake = m_intake;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setRollerSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.StopRollers();
    }
}

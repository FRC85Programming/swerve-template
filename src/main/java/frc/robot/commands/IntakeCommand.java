package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    private final Boolean m_output;

    public IntakeCommand(IntakeSubsystem m_intake, boolean output){
        this.m_intake = m_intake;
        this.m_output = output;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        // m_intake.setRollerSpeed(m_leftTriggerSupplier.getAsDouble());
        if (m_output){
            m_intake.setRollerSpeed(0.8);
        } else {
            m_intake.setRollerSpeed(-0.8);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.StopRollers();
    }
}

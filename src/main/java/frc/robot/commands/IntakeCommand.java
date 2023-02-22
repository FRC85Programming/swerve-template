package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    private final DoubleSupplier m_leftTriggerSupplier;

    public IntakeCommand(IntakeSubsystem m_intake,
                        DoubleSupplier leftTriggerSupplier){
        this.m_intake = m_intake;
        this.m_leftTriggerSupplier = leftTriggerSupplier;


        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setRollerSpeed(m_leftTriggerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.StopRollers();
    }
}

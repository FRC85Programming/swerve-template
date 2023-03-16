package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final DoubleSupplier m_Speed;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier speed) {
        this.m_intake = intake;
        this.m_Speed = speed;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setRollerSpeed(m_Speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.StopRollers();
    }
}

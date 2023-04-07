package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    Timer m_timer;
    Boolean timerStarted = false;
    Boolean intakeMode = false;
    double time;
    double intakePower;

    public TimedIntakeCommand(IntakeSubsystem intake, Boolean intakeMode, double time, double intakePower)
    {
        m_intakeSubsystem = intake;
        this.intakeMode = intakeMode;
        m_timer = new Timer();
        addRequirements(intake);
        this.time = time;
        this.intakePower = intakePower;
    }

    @Override
    public void execute() {
        if (timerStarted == false) {
            m_timer.start();
            timerStarted = true;
        }
        if (this.intakeMode) {
            m_intakeSubsystem.setRollerSpeed(() -> -0.8);
        } else {
            m_intakeSubsystem.setRollerSpeed(() -> intakePower);
        }

    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= time;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setRollerSpeed(() -> 0);
        timerStarted = false;
        m_timer.reset();
        intakeMode = false;
    }
}

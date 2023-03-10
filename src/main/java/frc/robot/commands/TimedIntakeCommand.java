package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends CommandBase
 {
    private final IntakeSubsystem m_intakeSubsystem;
    Timer m_timer;
    Boolean timerStarted = false;
    Boolean intakeMode = false;

    public TimedIntakeCommand(IntakeSubsystem intake, Boolean intakeMode)
    {
        m_intakeSubsystem = intake;
        this.intakeMode = intakeMode;
        m_timer = new Timer();
        addRequirements(intake);
    }

    @Override
    public void execute()
    {
        if (timerStarted == false) {
            m_timer.start();
            timerStarted = true;
        }
        if (intakeMode = true) {
            m_intakeSubsystem.setRollerSpeed(() -> -0.8);
        } else {
            m_intakeSubsystem.setRollerSpeed(() -> 0.6);
        }

    }
    @Override
    public boolean isFinished() {
        return m_timer.get() >= 1;
    }
    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setRollerSpeed(() -> 0);
        timerStarted = false;
        m_timer.reset();
        intakeMode = false;
    }
}


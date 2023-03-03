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
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    Timer m_timer;
    Boolean timerStarted = false;

    public TimedIntakeCommand(DrivetrainSubsystem driveTrain, ExtendoSubsystem extendo, IntakeSubsystem intake)
    {
        m_drivetrainSubsystem = driveTrain;
        m_extendoSubsystem = extendo;
        m_intakeSubsystem = intake;
        m_timer = new Timer();
        addRequirements(m_drivetrainSubsystem, extendo, intake);
    }

    @Override
    public void execute()
    {
        if (timerStarted == false) {
            m_timer.start();
            timerStarted = true;
        }
        m_intakeSubsystem.setRollerSpeed(() -> 0.8);

    }
    @Override
    public boolean isFinished() {
        return m_timer.get() >= 2;
    }
    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setRollerSpeed(() -> 0);
        timerStarted = false;
        m_timer.reset();
    }
}


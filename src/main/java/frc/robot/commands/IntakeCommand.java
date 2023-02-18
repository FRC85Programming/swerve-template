package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setRollerSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        intake.StopRollers();
    }
}

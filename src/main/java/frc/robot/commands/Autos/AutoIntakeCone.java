package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCone extends ExtendCommand
 {
    public AutoIntakeCone(ExtendoSubsystem extendo) {
        super(extendo, () -> 107, () -> 21.8, () -> -39.3, false, true);
    }

    @Override
    public void execute() {
        super.execute();
    }

    public boolean isFinished() {
        return super.isFinished();
    }
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
 }




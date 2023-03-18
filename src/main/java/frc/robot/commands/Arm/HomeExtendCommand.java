package frc.robot.commands.Arm;

import frc.robot.subsystems.ExtendoSubsystem;

public class HomeExtendCommand extends ExtendCommand {

    public HomeExtendCommand(ExtendoSubsystem extendo) {
        super(extendo, () -> -10, () -> -10, () -> 10, true);
    }
    
}

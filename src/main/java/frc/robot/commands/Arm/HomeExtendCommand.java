package frc.robot.commands.Arm;

import frc.robot.subsystems.ExtendoSubsystem;

public class HomeExtendCommand extends ExtendCommand {
    private ExtendoSubsystem m_ExtendoSubsystem;

    public HomeExtendCommand(ExtendoSubsystem extendo) {
        super(extendo, () -> -100, () -> -100, () -> 100, true);
        m_ExtendoSubsystem = extendo;
    }
    
    @Override
    public boolean isFinished() {
        return m_ExtendoSubsystem.allAxesHome();
    }
}

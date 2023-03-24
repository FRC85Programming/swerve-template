package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExtendoSubsystem;

public class ExtendPauseCommand extends SequentialCommandGroup {
    private final ExtendoSubsystem m_extendoSubsystem;

    public ExtendPauseCommand(ExtendoSubsystem extendo) {
        m_extendoSubsystem = extendo;
        addCommands(
                new ExtendCommand(m_extendoSubsystem,
                        () -> 5,
                        () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
                        () -> SmartDashboard.getNumber("DesiredWristPosition", 0), false, true),
                new ExtendCommand(m_extendoSubsystem,
                        () -> SmartDashboard.getNumber("DesiredExtendPosition", 0),
                        () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
                        () -> SmartDashboard.getNumber("DesiredWristPosition", 0), false, false));
    }

}

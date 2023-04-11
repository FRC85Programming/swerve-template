package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExtendoSubsystem;

public class ExtendPauseCommand extends SequentialCommandGroup {
    private final ExtendoSubsystem m_extendoSubsystem;

    public ExtendPauseCommand(ExtendoSubsystem extendo) {
        m_extendoSubsystem = extendo;
        double desiredExtend = SmartDashboard.getNumber("DesiredExtendPosition", 0);
        String positionName = SmartDashboard.getString(("DesiredPositionName"), "");

        if (positionName.toLowerCase().equals("home")) {
                addCommands(
                        new ExtendCommand(m_extendoSubsystem,
                                () -> desiredExtend,
                                () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
                                () -> SmartDashboard.getNumber("DesiredWristPosition", 0), false, false));
        } else {
                addCommands(
                        new ExtendCommand(m_extendoSubsystem,
                                null,
                                () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
                                () -> SmartDashboard.getNumber("DesiredWristPosition", 0), false, true),
                        new ExtendCommand(m_extendoSubsystem,
                                () -> desiredExtend,
                                () -> SmartDashboard.getNumber("DesiredPivotPosition", 0),
                                () -> SmartDashboard.getNumber("DesiredWristPosition", 0), false, false));
        }
    }

}

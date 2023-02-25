package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class CS extends SequentialCommandGroup {
    private final RobotContainer m_robotContainer;
    public CS(DrivetrainSubsystem driveTrain, RobotContainer robotContainer) {
        this.m_robotContainer = robotContainer;
        addCommands(
            m_robotContainer.getAutonomousCommand("PW_CS"),
            new WaitCommand(.5),
            m_robotContainer.getAutonomousCommand("PW_CSPart2"),
            new AutoLevelCommand(driveTrain)
        );
    }
}

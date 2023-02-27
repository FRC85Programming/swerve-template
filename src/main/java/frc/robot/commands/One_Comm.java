package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class One_Comm extends SequentialCommandGroup {
    private final RobotContainer m_robotContainer;
    private final ExtendoSubystem m_ExtendoSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    public One_Comm(DrivetrainSubsystem driveTrain, RobotContainer robotContainer, double extendPosition, double pivotAngle, double intakeWrist, boolean output){
      this.m_robotContainer = robotContainer;
      addCommands(
        new PivotCommand(m_ExtendoSubsystem, 0.1),
        new ExtendCommand(m_ExtendoSubsystem, intakeSubsystem, extendPosition, pivotAngle, intakeWrist),
        new IntakeCommand(intakeSubsystem, output),
        m_robotContainer.getAutonomousCommand("PW_1_Comm")
      );
    }
}  
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class One_Comm_CS_L extends SequentialCommandGroup {
    private final RobotContainer m_robotContainer;
    private final ExtendoSubsystem m_ExtendoSubsystem;
    public One_Comm_CS_L(DrivetrainSubsystem driveTrain, RobotContainer robotContainer, double extendPosition, double pivotAngle, double intakeWrist, boolean output,
    ExtendoSubsystem extendoSubsystem, IntakeSubsystem intakeSubsystem){
      this.m_robotContainer = robotContainer;
      this.m_ExtendoSubsystem = extendoSubsystem;
      addCommands(
        new PivotCommand(m_ExtendoSubsystem),
        new ExtendCommand(m_ExtendoSubsystem, () -> extendPosition, () -> pivotAngle, () -> intakeWrist),
        new IntakeCommand(intakeSubsystem, () -> 0.8),
        m_robotContainer.getAutonomousCommand(),
        new WaitCommand(.5),
        //m_robotContainer.getAutonomousCommand("PW_1_Comm_CS_LPart2"),
        new AutoLevelCommand(driveTrain)
      );
    }
} 

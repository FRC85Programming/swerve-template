package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.IntakeCommand;
import frc.robot.commands.Arm.PivotCommand;
import frc.robot.subsystems.*;

public class One_Comm_R extends SequentialCommandGroup {
  private final RobotContainer m_robotContainer;
  private final ExtendoSubsystem m_ExtendoSubsystem;

  public One_Comm_R(DrivetrainSubsystem driveTrain, RobotContainer robotContainer, double extendPosition,
      double pivotAngle, double intakeWrist, boolean output,
      ExtendoSubsystem extendoSubsystem, IntakeSubsystem intakeSubsystem) {
    this.m_robotContainer = robotContainer;
    this.m_ExtendoSubsystem = extendoSubsystem;
    addCommands(
        new PivotCommand(m_ExtendoSubsystem),
        new ExtendCommand(m_ExtendoSubsystem, () -> extendPosition, () -> pivotAngle, () -> intakeWrist),
        new IntakeCommand(intakeSubsystem, () -> 0.8),
        m_robotContainer.getAutonomousCommand());
  }
}
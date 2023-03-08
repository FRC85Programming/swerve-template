package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Comm_R extends SequentialCommandGroup {
  private final RobotContainer m_robotContainer;

  public Comm_R(DrivetrainSubsystem driveTrain, RobotContainer robotContainer) {
    this.m_robotContainer = robotContainer;
    addCommands(
    // m_robotContainer.getAutonomousCommand("PW_Comm_R")
    );
  }
}

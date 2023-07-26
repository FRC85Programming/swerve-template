package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionTracking;

public class PathAndCubeMid extends ParallelCommandGroup {

    public PathAndCubeMid(RobotContainer m_robotContainer, DrivetrainSubsystem m_DrivetrainSubsystem, VisionTracking vision, ExtendoSubsystem m_ExtendoSubsystem, IntakeSubsystem m_IntakeSubsystem, PathPlannerTrajectory path, Boolean isFirstPath){
        addCommands(
            new FollowPathCommand(m_robotContainer, path, isFirstPath),
            new AutoScore(m_ExtendoSubsystem, "cube middle")
        );
    }

}
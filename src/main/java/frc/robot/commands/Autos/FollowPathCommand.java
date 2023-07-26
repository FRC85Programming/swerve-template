package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FollowPathCommand extends CommandBase {
    private final RobotContainer m_robotContainer;
    private final PathPlannerTrajectory path;
    private final Boolean isFirstPath;

    public FollowPathCommand(RobotContainer m_robotContainer, PathPlannerTrajectory path, Boolean isFirstPath) {
        this.m_robotContainer = m_robotContainer;
        this.isFirstPath = isFirstPath;
        this.path = path;

    }

    @Override
    public void execute() {
        // m_ExtendoSubsystem.PivotArmTelescope(0.1);
        m_robotContainer.followTrajectoryCommand(path, isFirstPath);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
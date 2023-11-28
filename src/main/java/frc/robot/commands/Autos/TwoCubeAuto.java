package frc.robot.commands.Autos;

import frc.robot.RobotContainer;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class TwoCubeAuto extends SequentialCommandGroup {

    PathPlannerTrajectory NodeToCube = PathPlanner.loadPath("NodeToCube", new PathConstraints(3, 2));
    PathPlannerTrajectory CubeToNode = PathPlanner.loadPath("CubeToNode", new PathConstraints(3, 2));

    public TwoCubeAuto(RobotContainer m_robotContainer, DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new HomeExtendCommand(extendo),
            new TimedIntakeCommand(intake, true, 0.5, -0.8),
            new AutoScore(extendo, "cube high"),
            new AutoScoreExtend(extendo, "cube high"),
            new TimedIntakeCommand(intake, false, 0.5, 0.8),
            new PathAndCubeIntake(m_robotContainer, driveTrain, vision, extendo, intake, NodeToCube, true),
            new PathAndCubeMid(m_robotContainer, driveTrain, vision, extendo, intake, CubeToNode, false),
            new AutoScoreExtend(extendo, "cube middle"),
            new TimedIntakeCommand(intake, false, 1, 0.8)
        );
    }
}

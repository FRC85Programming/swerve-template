package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Chassis.DriveAndIntake;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;

public class ScoreAndPickup extends SequentialCommandGroup {

    public ScoreAndPickup(DrivetrainSubsystem driveTrain, VisionTracking vision, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        addCommands(
            new zeroWheels(driveTrain),
            new ZeroGyroscopeCommand(driveTrain, 0),
            new AutoScore(extendo, "cone mid"),
            new TimedIntakeCommand(intake, false),
            new StrafeAndHome(driveTrain, vision, extendo, intake, 0.5),
            new DriveDistance(driveTrain, vision, 0, -2, 1, 5, 180, false),
            new AutoIntakeCube(extendo, intake),
            new DriveAndIntake(driveTrain, vision, extendo, intake, 2),
            new DriveAndHomeCommand(driveTrain, vision, extendo, intake, 6),
            new AutoScore(extendo, "cube mid"),
            new TimedIntakeCommand(intake, null),
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0)


            
        );
    }
}

 
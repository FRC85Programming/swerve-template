package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Chassis.AutoLevelCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.zeroWheels;
import frc.robot.subsystems.*;

public class CubeIntakePositionIntoCubeIntake extends SequentialCommandGroup {
    public CubeIntakePositionIntoCubeIntake(DrivetrainSubsystem driveTrain, VisionTracking vision, IntakeSubsystem intake, ExtendoSubsystem extendo) {
        addCommands(  
            new AutoIntakeCube(extendo),
            new TimedIntakeCommand(intake, false, 3, 0.8)
        );
    }
}
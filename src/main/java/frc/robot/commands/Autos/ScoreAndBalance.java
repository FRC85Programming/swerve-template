package frc.robot.commands.Autos;

import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.BrakeWheelsCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreAndBalance  extends SequentialCommandGroup{

        public String scorePosition;
    
        public ScoreAndBalance(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake, String scorePosition) {

            addCommands(
                    new ZeroGyroscopeCommand(driveTrain, 180),
                    new ZeroPitchRollCommand(driveTrain),
                    new HomeExtendCommand(extendo),
                    new TimedIntakeCommand(intake, true, 0.51),
                    new AutoScore(extendo, scorePosition),
                    new AutoScoreExtend(extendo, scorePosition),
                    new TimedIntakeCommand(intake, false, 1.5),
                    new HomeExtendCommand(extendo),
                    new DriveDistance(driveTrain, vision, 0, -1.7, 0, 0, 0, false),
                    new WaitCommand(0.1),
                    new DriveDistance(driveTrain, vision, 0, 1.5, 0, -2.5, 0, false),
                    new AutoLevelPIDCommand(driveTrain),
                    new BrakeWheelsCommand(driveTrain)

            );
        }
    }
package frc.robot.commands.Autos;

import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.commands.Chassis.AutoLevelPIDCommand;
import frc.robot.commands.Chassis.BrakeWheelsCommand;
import frc.robot.commands.Chassis.DriveDistance;
import frc.robot.commands.Chassis.ZeroGyroscopeCommand;
import frc.robot.commands.Chassis.ZeroPitchRollCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ExtendCommand;

public class BalanceAuto extends SequentialCommandGroup{
    
        public BalanceAuto(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {

            SmartDashboard.putBoolean("BalanceAuto started", true);
            addCommands(
                new ZeroGyroscopeCommand(driveTrain, 180),
                new ZeroPitchRollCommand(driveTrain),
                new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
                new DriveDistance(driveTrain, vision, 0, -0.85, 0, -4.5),
                new WaitCommand(0.1),
                new DriveDistance(driveTrain, vision, 0, 0.85, 0, -3),
                new AutoLevelPIDCommand(driveTrain),
                new BrakeWheelsCommand(driveTrain)
            );
        }
    }

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuto extends SequentialCommandGroup{
        private final DrivetrainSubsystem m_drivetrainSubsystem;
        private final ExtendoSubsystem m_extendoSubsystem;
        private final IntakeSubsystem m_intakeSubsystem;
    
        public BalanceAuto(DrivetrainSubsystem driveTrain, VisionTracking vision, ExtendoSubsystem extendo, IntakeSubsystem intake) {
            m_drivetrainSubsystem = driveTrain;
            m_extendoSubsystem = extendo;
            m_intakeSubsystem = intake;
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

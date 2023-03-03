package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuto extends SequentialCommandGroup{
        private final DrivetrainSubsystem m_drivetrainSubsystem;
        private final ExtendoSubsystem m_extendoSubsystem;
        private final IntakeSubsystem m_intakeSubsystem;
    
        public BalanceAuto(DrivetrainSubsystem driveTrain, ExtendoSubsystem extendo, IntakeSubsystem intake) {
            m_drivetrainSubsystem = driveTrain;
            m_extendoSubsystem = extendo;
            m_intakeSubsystem = intake;
            SmartDashboard.putBoolean("BalanceAuto started", true);
            addCommands(
                new ZeroGyroscopeCommand(driveTrain),
                new ZeroPitchRollCommand(driveTrain),
                new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
                new DriveDistance(driveTrain, 0, -0.8, 0, 5),
                new WaitCommand(0.3),
                new DriveDistance(driveTrain, 0, 0.8, 0, -3),
                new AutoLevelPIDCommand(driveTrain),
                new BrakeWheelsCommand(driveTrain)
            );
        }
    }

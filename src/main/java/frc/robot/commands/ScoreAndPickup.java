package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ScoreAndPickup extends SequentialCommandGroup {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public ScoreAndPickup(DrivetrainSubsystem driveTrain, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        m_drivetrainSubsystem = driveTrain;
        m_extendoSubsystem = extendo;
        m_intakeSubsystem = intake;
        addCommands(  
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new ZeroGyroscopeCommand(driveTrain, 180),
            new AutoScore(driveTrain, extendo, intake),
            new TimedIntakeCommand(driveTrain, extendo, intake),
            new DriveAndHomeCommand(m_drivetrainSubsystem, m_extendoSubsystem, m_intakeSubsystem,SmartDashboard.getNumber("Drive and Home Distance", 0)),
            new RotateAndIntakePosition(driveTrain, extendo, intake)
        );
    }
}

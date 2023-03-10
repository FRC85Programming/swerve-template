package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ManualOnePlace extends SequentialCommandGroup {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public ManualOnePlace(DrivetrainSubsystem driveTrain, RobotContainer robotContainer, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        m_drivetrainSubsystem = driveTrain;
        m_extendoSubsystem = extendo;
        m_intakeSubsystem = intake;
        addCommands(  
            new ZeroGyroscopeCommand(driveTrain, 180),
            new AutoScore(driveTrain, extendo),
            new TimedIntakeCommand(intake, false),
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new DriveDistance(driveTrain, 0, -1.0, 0.0, 4.95, 0)
        );
    }
}

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreBalanceAuto extends SequentialCommandGroup {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public ScoreBalanceAuto(DrivetrainSubsystem driveTrain, ExtendoSubsystem extendo, IntakeSubsystem intake) {
        m_drivetrainSubsystem = driveTrain;
        m_extendoSubsystem = extendo;
        m_intakeSubsystem = intake;

        addCommands(
            new ZeroGyroscopeCommand(driveTrain),
            new ZeroPitchRollCommand(driveTrain),
            new AutoScore(driveTrain, extendo, intake),
            new TimedIntakeCommand(driveTrain, extendo, intake),
            new ExtendCommand(extendo, () -> 0, () -> 0, () -> 0),
            new DriveDistance(driveTrain, 0, -0.8, 0, 5.2),
            new WaitCommand(0.3),
            new DriveDistance(driveTrain, 0.8, 0, 0, 0.5),
            new AutoLevelCommand(driveTrain)
        );
    }
}

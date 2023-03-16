package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCube extends CommandBase
 {
    private final ExtendoSubsystem m_extendoSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;


    public AutoIntakeCube(ExtendoSubsystem extendo, IntakeSubsystem intake)
    {
        m_extendoSubsystem = extendo;
        m_intakeSubsystem = intake;

        addRequirements(extendo, intake);
    }

    @Override
    public void execute()
    {
        if (m_extendoSubsystem.getPivotAngle() <= 14) {
            m_extendoSubsystem.Pivot(0.85, 0);
        } else {
            m_extendoSubsystem.Pivot(0, 0);
        }
        if (m_extendoSubsystem.getExtendPosition() <= 11) {
            m_extendoSubsystem.ExtendTelescope(0.25, 0);
        } else {
            m_extendoSubsystem.ExtendTelescope(0, 0);
        }
        if (m_extendoSubsystem.getIntakeWrist() >= -27) {
            m_extendoSubsystem.Wrist(-0.3, 0);
        } else {
            m_extendoSubsystem.Wrist(0, 0);
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);

        m_intakeSubsystem.StopRollers();*/

    }

    public boolean isFinished() {
        return m_extendoSubsystem.getPivotAngle() >= 8 && m_extendoSubsystem.getExtendPosition() >= 5 && m_extendoSubsystem.getIntakeWrist() <= -24;
    }
    public void end(boolean interrupted) {
        m_extendoSubsystem.Pivot(0, 0);
        m_extendoSubsystem.ExtendTelescope(0, 0);
        m_extendoSubsystem.Wrist(0, 0);
        SmartDashboard.putNumber("z Command Done", 1);
    }
}


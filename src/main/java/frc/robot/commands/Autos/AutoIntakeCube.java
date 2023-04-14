package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.ExtendCommand;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCube extends ExtendCommand
 {
    //private final ExtendoSubsystem m_extendoSubsystem;

    public AutoIntakeCube(ExtendoSubsystem extendo)
    {
        super(extendo, () -> 75, () -> 21, () -> -22, false, true);
       // m_extendoSubsystem = extendo;
        
        //addRequirements(extendo);
    }

    @Override
    public void execute()
    {
        super.execute();

        /*if (m_extendoSubsystem.getPivotAngle() <= 21) {
            m_extendoSubsystem.Pivot(1, 0);
        } else {
            m_extendoSubsystem.Pivot(0, 0);
        }
        if (m_extendoSubsystem.getExtendPosition() <= 108) {
            m_extendoSubsystem.ExtendTelescope(0.25, 0);
        } else {
            m_extendoSubsystem.ExtendTelescope(0, 0);
        }
        if (m_extendoSubsystem.getIntakeWrist() >= -40) {
            m_extendoSubsystem.Wrist(-0.3, 0);
        } else {
            m_extendoSubsystem.Wrist(0, 0);
        }
        /*m_intakeSubsystem.setRollerSpeed(-0.8);

        m_intakeSubsystem.StopRollers();*/

    }

    public boolean isFinished() {
        return super.isFinished();
    }
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
 }




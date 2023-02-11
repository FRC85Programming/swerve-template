package frc.robot.commands;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class ExtendCommand extends CommandBase{
    private final ExtendoSubystem m_ExtendoSubystem;
    private final boolean direction;

    public ExtendCommand(ExtendoSubystem extendo, boolean direction){
        this.m_ExtendoSubystem = extendo;
        this.direction = direction;

        addRequirements(extendo);
    }

@Override
public void initialize() {
    if(direction){
        m_ExtendoSubystem.ExtendTelescope(0.1);
    } else {
        m_ExtendoSubystem.ExtendTelescope(-0.1);
    }
    

}

@Override
public void end(boolean interrupted) {
    m_ExtendoSubystem.ExtendTelescope(0.0);
}
}

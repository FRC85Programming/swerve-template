package frc.robot.commands;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class ExtendCommand extends CommandBase{
    private final ExtendoSubystem m_ExtendoSubystem;

    public ExtendCommand(ExtendoSubystem extendo){
        this.m_ExtendoSubystem = extendo;
    }

@Override
public void initialize() {
    m_ExtendoSubystem.ExtendTelescope(0.1);
}

@Override
public void end(boolean interrupted) {
    m_ExtendoSubystem.ExtendTelescope(0.0);
}
}

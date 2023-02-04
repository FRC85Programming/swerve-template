package frc.robot.commands;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class ExtendCommand extends CommandBase{
    ExtendoSubystem extendo;

    public ExtendCommand(ExtendoSubystem extendo){
        this.extendo = extendo;
    }

int setPoint = 0;
@Override
public void initialize() {
    extendo.ExtendTelescope(setPoint);
}
}

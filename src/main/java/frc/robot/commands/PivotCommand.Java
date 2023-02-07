package frc.robot.commands;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendoSubystem;

public class PivotCommand extends CommandBase{
    PivotCommand Pivot;

    public PivotCommand(PivotCommand Pivot){
        this.Pivot = Pivot;
    }

    @Override
    public void initialize() {
        pivot.PivotTelescope();
    }   
}

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendAndIntakeCommand extends ParallelCommandGroup {
    public ExtendAndIntakeCommand(ExtendoSubsystem extendo, IntakeSubsystem intake, DoubleSupplier extendPosition, DoubleSupplier pivotPosition, DoubleSupplier wristPosition, DoubleSupplier rollerSpeed) {
        addCommands(new ExtendCommand(extendo, extendPosition, pivotPosition, wristPosition), new IntakeCommand(intake, rollerSpeed));
    }
}

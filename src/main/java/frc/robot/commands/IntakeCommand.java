package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class IntakeCommand extends CommandBase{
    IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setRollerSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        intake.StopRollers();
    }
}

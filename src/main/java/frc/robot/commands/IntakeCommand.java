package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class IntakeCommand extends CommandBase{
    IntakeSubsystem intake;
    double speed;

    public IntakeCommand(IntakeSubsystem intake, double speed){
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setRollerSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSpeed(0);
    }
}

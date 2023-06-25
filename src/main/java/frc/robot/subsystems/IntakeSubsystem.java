package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeRollerMotor = new CANSparkMax(Constants.INTAKE_ROLLERS_MOTOR, MotorType.kBrushless);

    public IntakeSubsystem() {

    }

    public void setRollerSpeed(DoubleSupplier speed) {
        intakeRollerMotor.set(speed.getAsDouble());
    }

    public void StopRollers() {
        intakeRollerMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }
}

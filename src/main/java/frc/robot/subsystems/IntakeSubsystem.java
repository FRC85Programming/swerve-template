package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeRollerMotor = new CANSparkMax(Constants.INTAKE_ROLLERS_MOTOR, MotorType.kBrushless);
    private final CANSparkMax intakePivotMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    private final DigitalInput intakePivotLimitSwitch = new DigitalInput(Constants.INTAKE_PIVOT_LIMIT_SWITCH);

    public void setRollerSpeed() {
        double speed = SmartDashboard.getNumber("Roller Speed", 1);
        intakeRollerMotor.set(speed);
    }

    public void StopRollers() {
        intakeRollerMotor.set(0);
    }

    public void Pivot(double speed) {
        if (speed < 0) {
            if (intakePivotMotor.getEncoder().getPosition() > 50) {
                intakePivotMotor.set(0);
            } else {
                intakePivotMotor.set(speed * 0.1);
            }
        } else {
            if (intakePivotLimitSwitch.get()) {
                intakePivotMotor.getEncoder().setPosition(0);
                intakePivotMotor.set(0);
            } else {
                intakePivotMotor.set(speed * 0.1);
            }
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("intake wrist limit sensor", intakePivotLimitSwitch.get());
        SmartDashboard.putNumber("Intake wrist position", intakePivotMotor.getEncoder().getPosition());
    }
}

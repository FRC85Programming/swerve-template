package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeRollerMotor = new CANSparkMax(Constants.INTAKE_ROLLERS_MOTOR, MotorType.kBrushless);
    private final CANSparkMax intakePivotMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    private final DigitalInput intakePivotLimitSwitch = new DigitalInput(Constants.INTAKE_PIVOT_LIMIT_SWITCH);
    private final double WristSpeedScale = 0.25;

    public IntakeSubsystem() {
        
    }

    public void setRollerSpeed(Double speed) {
        intakeRollerMotor.set(speed);
    }

    public void StopRollers() {
        intakeRollerMotor.stopMotor();
    }

    public void Pivot(double speed) {
        if (speed < 0) {
            if (intakePivotMotor.getEncoder().getPosition() < -82) {
                intakePivotMotor.stopMotor();
            } else {
                intakePivotMotor.set(speed * WristSpeedScale);
            }
        } else if (speed > 0) {
            if (intakePivotLimitSwitch.get()) {
                intakePivotMotor.getEncoder().setPosition(0);
                intakePivotMotor.stopMotor();
            } else {
                intakePivotMotor.set(speed * WristSpeedScale);
            }
        } else {
            intakePivotMotor.stopMotor();
        }
    }

    public double getIntakeWrist(){
        return intakePivotMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("intake wrist limit sensor", intakePivotLimitSwitch.get());
        SmartDashboard.putNumber("Intake wrist position", intakePivotMotor.getEncoder().getPosition());
    }
}

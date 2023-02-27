package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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
    private final PIDController intakePID = new PIDController(0, 0, 0);
    private final double WristSpeedScale = 0.50;

    public IntakeSubsystem() {
        
    }

    public void setRollerSpeed(Double speed) {
        intakeRollerMotor.set(speed);
    }

    public void StopRollers() {
        intakeRollerMotor.stopMotor();
    }

    public void Pivot(double speed, double desiredPosition) {
        double kp = SmartDashboard.getNumber("kp Intake", 0);
        double ki = SmartDashboard.getNumber("ki Intake", 0);
        double kd = SmartDashboard.getNumber("kd Intake", 0);

        intakePID.setPID(kp, ki, kd);

        if (desiredPosition != 0){
            speed = intakePID.calculate(intakePivotMotor.getEncoder().getPosition(), desiredPosition);
        }
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

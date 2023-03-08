// package frc.robot.subsystems;
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubsystem extends SubsystemBase {
    // Wrist stuff
    private final CANSparkMax intakeWristMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    private final DigitalInput intakeWristLimitSwitch = new DigitalInput(Constants.INTAKE_PIVOT_LIMIT_SWITCH);
    private final PIDController WristPID = new PIDController(0, 0, 0);
    private final double WristSpeedScale = 0.80;

    // extendo pivot
    private final CANSparkMax extendExtendoMotor = new CANSparkMax(Constants.EXTENDO_EXTEND_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax pivotTelescopeArmMotor = new CANSparkMax(Constants.EXTENDO_ARM_PIVOT_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax pivotTelescopeArmMotorTwo = new CANSparkMax(Constants.EXTENDO_ARM_PIVOT_MOTOR_TWO, MotorType.kBrushless);
    private final DigitalInput PivotArmLimitSwitch = new DigitalInput(Constants.EXTENDO_PIVOT_LIMIT_SWITCH);
    private final DigitalInput ExtendLimitSwitch = new DigitalInput(Constants.EXTENDO_EXTEND_LIMIT_SWITCH);
    private final PIDController pivotPID = new PIDController(0, 0, 0);
    private final PIDController extendoPID = new PIDController(0, 0, 0);
    private final double extendSpeedScale = 0.4;
    private final double pivotSpeedScale = 0.5;
    private  double maxPivot = 87;

    public ExtendoSubsystem() {
        extendExtendoMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeArmMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeArmMotorTwo.setIdleMode(IdleMode.kBrake);

        pivotTelescopeArmMotor.setInverted(true);
        pivotTelescopeArmMotorTwo.setInverted(true);

        SmartDashboard.putNumber("Max Pivot", maxPivot);
    }

    public void ExtendTelescope(double speed, double desiredPosition) {
        double kp = SmartDashboard.getNumber("kp Extendo", 1);
        double ki = SmartDashboard.getNumber("ki Extendo", 0);
        double kd = SmartDashboard.getNumber("kd Extendo", 0);

        extendoPID.setPID(kp, ki, kd);

        if (desiredPosition != 0) {
            speed = extendoPID.calculate(extendExtendoMotor.getEncoder().getPosition(), desiredPosition);
        }
        if (speed > 0) {
            if (extendExtendoMotor.getEncoder().getPosition() > 135) {
                // stops motor with upper limit switch
                extendExtendoMotor.stopMotor();
            } else {
                // doesnt stop if limit switch isnt pressed
                extendExtendoMotor.set(speed * extendSpeedScale);
            }
        } else if (speed < 0) {
            if (ExtendLimitSwitch.get()) {
                extendExtendoMotor.getEncoder().setPosition(0);
                extendExtendoMotor.stopMotor();
            } else {
                extendExtendoMotor.set(speed * extendSpeedScale);
            }
        } else {
            extendExtendoMotor.stopMotor();
        }
    }

    public void Pivot(double speed, double desiredPosition) {
        double kp = SmartDashboard.getNumber("kp Pivot", 1);
        double ki = SmartDashboard.getNumber("ki Pivot", 0);
        double kd = SmartDashboard.getNumber("kd Pivot", 0);

        pivotPID.setPID(kp, ki, kd);

        if (desiredPosition != 0) {
            speed = pivotPID.calculate(pivotTelescopeArmMotor.getEncoder().getPosition(), desiredPosition);
        }

        if (speed > 0) {
            if (pivotTelescopeArmMotor.getEncoder().getPosition() > maxPivot) {
                pivotTelescopeArmMotor.stopMotor();
                pivotTelescopeArmMotorTwo.stopMotor();
            } else {
                pivotTelescopeArmMotor.set(speed * pivotSpeedScale);
                pivotTelescopeArmMotorTwo.set(speed * pivotSpeedScale);
            }
        } else if (speed < 0) {
            if (PivotArmLimitSwitch.get()) {
                pivotTelescopeArmMotor.getEncoder().setPosition(0);
                pivotTelescopeArmMotor.stopMotor();
                pivotTelescopeArmMotorTwo.getEncoder().setPosition(0);
                pivotTelescopeArmMotorTwo.stopMotor();
            } else if (pivotTelescopeArmMotor.getEncoder().getPosition() < 34 && extendExtendoMotor.getEncoder().getPosition() > 20) {
                pivotTelescopeArmMotor.stopMotor();
                pivotTelescopeArmMotorTwo.stopMotor();
            } else if (pivotTelescopeArmMotor.getEncoder().getPosition() < 20 && intakeWristMotor.getEncoder().getPosition() < -15) {
                pivotTelescopeArmMotor.stopMotor();
                pivotTelescopeArmMotorTwo.stopMotor();
            } else if (pivotTelescopeArmMotor.getEncoder().getPosition() < 7){
                pivotTelescopeArmMotor.set(speed * 0.2);
                pivotTelescopeArmMotor.set(speed * 0.2);
            } else {
                pivotTelescopeArmMotor.set(speed * pivotSpeedScale);
                pivotTelescopeArmMotorTwo.set(speed * pivotSpeedScale);
            }
        } else {
            pivotTelescopeArmMotorTwo.stopMotor();
            pivotTelescopeArmMotor.stopMotor();
        }
    }

    public double getExtendPosition() {
        return extendExtendoMotor.getEncoder().getPosition();
    }

    public double getPivotAngle() {
        return pivotTelescopeArmMotor.getEncoder().getPosition();
    }

    public void Wrist(double speed, double desiredPosition) {
        double kp = SmartDashboard.getNumber("kp Intake", 1);
        double ki = SmartDashboard.getNumber("ki Intake", 0);
        double kd = SmartDashboard.getNumber("kd Intake", 0);

        WristPID.setPID(kp, ki, kd);

        if (desiredPosition != 0){
            speed = WristPID.calculate(intakeWristMotor.getEncoder().getPosition(), desiredPosition);
        }
        if (speed < 0) {
            if (intakeWristMotor.getEncoder().getPosition() < -82) {
                intakeWristMotor.stopMotor();
            } else {
                intakeWristMotor.set(speed * WristSpeedScale);
            }
        } else if (speed > 0) {
            if (intakeWristLimitSwitch.get()) {
                intakeWristMotor.getEncoder().setPosition(0);
                intakeWristMotor.stopMotor();
            } else if (intakeWristMotor.getEncoder().getPosition() > -7){
                intakeWristMotor.set(speed * 0.2);
            } else {
                intakeWristMotor.set(speed * WristSpeedScale);
            }
        } else {
            intakeWristMotor.stopMotor();
        }
    }
    public double getIntakeWrist(){
        return intakeWristMotor.getEncoder().getPosition();
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Extendo pivot limit switch", PivotArmLimitSwitch.get());
        SmartDashboard.putBoolean("Extendo extend limit switch", ExtendLimitSwitch.get());
        SmartDashboard.putNumber("Extendo extend position", extendExtendoMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extendo pivot position", pivotTelescopeArmMotor.getEncoder().getPosition());

        SmartDashboard.putString("Extendo Pivot Brake mode", pivotTelescopeArmMotor.getIdleMode().toString());
        SmartDashboard.putString("Extendo extend Brake mode", extendExtendoMotor.getIdleMode().toString());

        SmartDashboard.putBoolean("intake wrist limit sensor", intakeWristLimitSwitch.get());
        SmartDashboard.putNumber("Intake wrist position", intakeWristMotor.getEncoder().getPosition());

        maxPivot = SmartDashboard.getNumber("Max Pivot", maxPivot);
    }
}
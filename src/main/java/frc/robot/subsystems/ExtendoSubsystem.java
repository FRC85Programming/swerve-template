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
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    private final DigitalInput wristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
    private final PIDController wristPID = new PIDController(0, 0, 0);
    private final double wristSpeedScale = 0.80;

    // extendo pivot
    private final CANSparkMax extendMotor = new CANSparkMax(Constants.EXTEND_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax pivotMotorTwo = new CANSparkMax(Constants.PIVOT_MOTOR_TWO, MotorType.kBrushless);
    private final DigitalInput pivotLimitSwitch = new DigitalInput(Constants.PIVOT_LIMIT_SWITCH);
    private final DigitalInput extendLimitSwitch = new DigitalInput(Constants.EXTEND_LIMIT_SWITCH);
    private final PIDController pivotPID = new PIDController(0, 0, 0);
    private final PIDController extendoPID = new PIDController(0, 0, 0);
    private final double extendSpeedScaleSafe = 0.4;
    private final double extendSpeedScale = 0.7;
    private final double pivotSpeedScale = 0.5;
    public static final double maxPivot = 80;
    public static final double maxExtend = 210;
    public static final double maxWrist = -64;
    private double pivotSafeZone = 10;
    private double extendSafeZone = 30;
    private double wristSafeZone = -15;
    private double wristSafeSpeed = 0.5;

    public ExtendoSubsystem() {
        wristMotor.setIdleMode(IdleMode.kBrake);
        extendMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotorTwo.setIdleMode(IdleMode.kBrake);

        wristMotor.setOpenLoopRampRate(0.4);
        extendMotor.setOpenLoopRampRate(0.0);
        pivotMotor.setOpenLoopRampRate(0.4);
        pivotMotorTwo.setOpenLoopRampRate(0.4);

        extendMotor.setInverted(false);
        wristMotor.setInverted(true);
        pivotMotor.setInverted(false);
        pivotMotorTwo.setInverted(false);

        wristMotor.burnFlash();
        extendMotor.burnFlash();
        pivotMotor.burnFlash();
        pivotMotorTwo.burnFlash();
    }

    public void ExtendTelescope(double speed, double desiredPosition) {
        ExtendTelescope(speed, desiredPosition, false);
    }

    public void ExtendTelescope(double speed, double desiredPosition, boolean enableZeroing) {
        if (SmartDashboard.getBoolean("Set extend PID", false)) {
            double kp = SmartDashboard.getNumber("kp Extendo", 1);
            double ki = SmartDashboard.getNumber("ki Extendo", 0);
            double kd = SmartDashboard.getNumber("kd Extendo", 0);
            extendoPID.setPID(kp, ki, kd);
            SmartDashboard.putBoolean("Set extend PID", false);
        }

        if (desiredPosition != 0) {
            speed = extendoPID.calculate(extendMotor.getEncoder().getPosition(), desiredPosition);
        }

        if (speed > 0) {
            if (extendMotor.getEncoder().getPosition() > maxExtend
                    || pivotMotor.getEncoder().getPosition() < pivotSafeZone) {
                extendMotor.stopMotor();
            } else {
                // doesnt stop if limit switch isnt pressed
                extendMotor.set(speed * extendSpeedScale);
            }
        } else if (speed < 0) {
            if (extendLimitSwitch.get()) {
                if (enableZeroing) {
                    extendMotor.getEncoder().setPosition(0);
                }
                extendMotor.stopMotor();
            } else if (extendMotor.getEncoder().getPosition() < extendSafeZone) {
                extendMotor.set(speed * extendSpeedScaleSafe);
            } else {
                extendMotor.set(speed * extendSpeedScale);
            }
        } else {
            extendMotor.stopMotor();
        }
    }

    public void Pivot(double speed, double desiredPosition) {
        Pivot(speed, desiredPosition, false);
    }

    public void Pivot(double speed, double desiredPosition, boolean enableZeroing) {
        if (SmartDashboard.getBoolean("Set pivot PID", false)) {
            double kp = SmartDashboard.getNumber("kp Pivot", 1);
            double ki = SmartDashboard.getNumber("ki Pivot", 0);
            double kd = SmartDashboard.getNumber("kd Pivot", 0);
            pivotPID.setPID(kp, ki, kd);
            SmartDashboard.putBoolean("Set pivot PID", false);
        }

        if (desiredPosition != 0) {
            speed = pivotPID.calculate(pivotMotor.getEncoder().getPosition(), desiredPosition);
        }

        if (pivotMotor.getEncoder().getPosition() < pivotSafeZone
                && wristMotor.getEncoder().getPosition() < wristSafeZone) {
            pivotMotor.stopMotor();
            pivotMotorTwo.stopMotor();
        } else if (speed > 0) {
            if (pivotMotor.getEncoder().getPosition() > maxPivot) {
                pivotMotor.stopMotor();
                pivotMotorTwo.stopMotor();
            } else {
                pivotMotor.set(speed * pivotSpeedScale);
                pivotMotorTwo.set(speed * pivotSpeedScale);
            }
        } else if (speed < 0) {
            if (pivotLimitSwitch.get()) {
                if (enableZeroing) {
                    pivotMotor.getEncoder().setPosition(0);
                    pivotMotorTwo.getEncoder().setPosition(0);
                }

                pivotMotor.stopMotor();
                pivotMotorTwo.stopMotor();
            } else if (pivotMotor.getEncoder().getPosition() < 20 && extendMotor.getEncoder().getPosition() > 20) {
                pivotMotor.stopMotor();
                pivotMotorTwo.stopMotor();
            } else if (pivotMotor.getEncoder().getPosition() < 7) {
                pivotMotor.set(speed * 0.2);
                pivotMotorTwo.set(speed * 0.2);
            } else {
                pivotMotor.set(speed * pivotSpeedScale);
                pivotMotorTwo.set(speed * pivotSpeedScale);
            }
        } else {
            pivotMotorTwo.stopMotor();
            pivotMotor.stopMotor();
        }
    }

    public double getExtendPosition() {
        return extendMotor.getEncoder().getPosition();
    }

    public double getPivotAngle() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void Wrist(double speed, double desiredPosition) {
        Wrist(speed, desiredPosition, false);
    }

    public void Wrist(double speed, double desiredPosition, boolean enableZeroing) {
        if (SmartDashboard.getBoolean("Set wrist PID", false)) {
            double kp = SmartDashboard.getNumber("kp Intake", 1);
            double ki = SmartDashboard.getNumber("ki Intake", 0);
            double kd = SmartDashboard.getNumber("kd Intake", 0);
            wristPID.setPID(kp, ki, kd);
            SmartDashboard.putBoolean("Set wrist PID", false);
        }

        if (desiredPosition != 0) {
            speed = wristPID.calculate(wristMotor.getEncoder().getPosition(), desiredPosition);
        }
        if (speed < 0) {
            if (wristMotor.getEncoder().getPosition() < maxWrist) {
                wristMotor.stopMotor();
            } else if (pivotMotor.getEncoder().getPosition() < pivotSafeZone) {
                if (wristMotor.getEncoder().getPosition() > -12) {
                    wristMotor.set(speed * wristSafeSpeed);
                } else {
                    wristMotor.stopMotor();
                }
            } else {
                wristMotor.set(speed * wristSpeedScale);
            }
        } else if (speed > 0) {
            if (wristLimitSwitch.get()) {
                if (enableZeroing) {
                    wristMotor.getEncoder().setPosition(0);
                }

                wristMotor.stopMotor();
            } else if (wristMotor.getEncoder().getPosition() > -14) {
                wristMotor.set(speed * 0.2);
            } else {
                wristMotor.set(speed * wristSpeedScale);
            }
        } else {
            wristMotor.stopMotor();
        }
    }

    public double getIntakeWrist() {
        return wristMotor.getEncoder().getPosition();
    }

    public void zeroEncodersIfLimits(){
        if (wristLimitSwitch.get()){
            wristMotor.getEncoder().setPosition(0);
        }

        if (extendLimitSwitch.get()){
            extendMotor.getEncoder().setPosition(0);
        }

        if (pivotLimitSwitch.get()){
            pivotMotor.getEncoder().setPosition(0);
        }
    }

    public boolean allAxesHome() {
        return pivotLimitSwitch.get() && extendLimitSwitch.get() && wristLimitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pivot limit switch", pivotLimitSwitch.get());
        SmartDashboard.putBoolean("Extend limit switch", extendLimitSwitch.get());
        SmartDashboard.putBoolean("Wrist limit switch", wristLimitSwitch.get());
        SmartDashboard.putNumber("Extend position", extendMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist position", wristMotor.getEncoder().getPosition());
    }
}

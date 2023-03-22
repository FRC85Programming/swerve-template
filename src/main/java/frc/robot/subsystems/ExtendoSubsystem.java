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
    private final CANSparkMax WristMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
    private final DigitalInput WristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
    private final PIDController WristPID = new PIDController(0, 0, 0);
    private final double WristSpeedScale = 0.80;

    // extendo pivot
    private final CANSparkMax extendMotor = new CANSparkMax(Constants.EXTEND_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax pivotMotorTwo = new CANSparkMax(Constants.PIVOT_MOTOR_TWO, MotorType.kBrushless);
    private final DigitalInput PivotLimitSwitch = new DigitalInput(Constants.PIVOT_LIMIT_SWITCH);
    private final DigitalInput ExtendLimitSwitch = new DigitalInput(Constants.EXTEND_LIMIT_SWITCH);
    private final PIDController pivotPID = new PIDController(0, 0, 0);
    private final PIDController extendoPID = new PIDController(0, 0, 0);
    private final double extendSpeedScale = 0.4;
    private final double pivotSpeedScale = 0.5;
    private double maxPivot = 80;
    private double maxExtend = 210;
    private double maxWrist = -64;
    private double pivotSafeZone = 10;
    private double extendSafeZone = 30;
    private double wristSafeZone = -15;
    private double wristSafeSpeed = 0.5;

    public ExtendoSubsystem() {
        WristMotor.setIdleMode(IdleMode.kBrake);
        extendMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotorTwo.setIdleMode(IdleMode.kBrake);

        WristMotor.setOpenLoopRampRate(0.2);
        extendMotor.setOpenLoopRampRate(0.1);
        pivotMotor.setOpenLoopRampRate(0.2);
        pivotMotorTwo.setOpenLoopRampRate(0.2);

        extendMotor.setInverted(false);
        WristMotor.setInverted(true);
        pivotMotor.setInverted(false);
        pivotMotorTwo.setInverted(false);

        WristMotor.burnFlash();
        extendMotor.burnFlash();
        pivotMotor.burnFlash();
        pivotMotorTwo.burnFlash();

        SmartDashboard.putNumber("Max Pivot", maxPivot);
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
                // stops motor with upper limit switch
                extendMotor.stopMotor();
            } else {
                // doesnt stop if limit switch isnt pressed
                extendMotor.set(speed * extendSpeedScale);
            }
        } else if (speed < 0) {
            if (ExtendLimitSwitch.get()) {
                if (enableZeroing) {
                    extendMotor.getEncoder().setPosition(0);
                }

                extendMotor.stopMotor();
            } else if (extendMotor.getEncoder().getPosition() < extendSafeZone) {
                extendMotor.set(speed * extendSpeedScale);
            } else {
                extendMotor.set(speed);
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
                && WristMotor.getEncoder().getPosition() < wristSafeZone) {
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
            if (PivotLimitSwitch.get()) {
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
            WristPID.setPID(kp, ki, kd);
            SmartDashboard.putBoolean("Set wrist PID", false);
        }

        if (desiredPosition != 0) {
            speed = WristPID.calculate(WristMotor.getEncoder().getPosition(), desiredPosition);
        }
        if (speed < 0) {
            if (WristMotor.getEncoder().getPosition() < maxWrist) {
                WristMotor.stopMotor();
            } else if (pivotMotor.getEncoder().getPosition() < pivotSafeZone) {
                if (WristMotor.getEncoder().getPosition() > -12) {
                    WristMotor.set(speed * wristSafeSpeed);
                } else {
                    WristMotor.stopMotor();
                }
            } else {
                WristMotor.set(speed * WristSpeedScale);
            }
        } else if (speed > 0) {
            if (WristLimitSwitch.get()) {
                if (enableZeroing) {
                    WristMotor.getEncoder().setPosition(0);
                }

                WristMotor.stopMotor();
            } else if (WristMotor.getEncoder().getPosition() > -14) {
                WristMotor.set(speed * 0.2);
            } else {
                WristMotor.set(speed * WristSpeedScale);
            }
        } else {
            WristMotor.stopMotor();
        }
    }

    public double getIntakeWrist() {
        return WristMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Extendo pivot limit switch", PivotLimitSwitch.get());
        SmartDashboard.putBoolean("Extendo extend limit switch", ExtendLimitSwitch.get());
        SmartDashboard.putNumber("Extendo extend position", extendMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extendo pivot position", pivotMotor.getEncoder().getPosition());

        SmartDashboard.putString("Extendo Pivot Brake mode", pivotMotor.getIdleMode().toString());
        SmartDashboard.putString("Extendo extend Brake mode", extendMotor.getIdleMode().toString());

        SmartDashboard.putBoolean("intake wrist limit sensor", WristLimitSwitch.get());
        SmartDashboard.putNumber("Intake wrist position", WristMotor.getEncoder().getPosition());

        maxPivot = SmartDashboard.getNumber("Max Pivot", maxPivot);
    }
}
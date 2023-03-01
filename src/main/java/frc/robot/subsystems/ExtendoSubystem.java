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

public class ExtendoSubystem extends SubsystemBase {
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
    private final IntakeSubsystem m_intake;

    public ExtendoSubystem(IntakeSubsystem intake) {
        extendExtendoMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeArmMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeArmMotorTwo.setIdleMode(IdleMode.kBrake);

        pivotTelescopeArmMotor.setInverted(true);
        pivotTelescopeArmMotorTwo.setInverted(true);
        m_intake = intake;
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
            if (pivotTelescopeArmMotor.getEncoder().getPosition() > 100) {
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
            } else if (pivotTelescopeArmMotor.getEncoder().getPosition() < 34 && extendExtendoMotor.getEncoder().getPosition() > 15) {
                pivotTelescopeArmMotor.stopMotor();
                pivotTelescopeArmMotorTwo.stopMotor();
            } else if (pivotTelescopeArmMotor.getEncoder().getPosition() < 20 && m_intake.getIntakeWrist() < -30) /*might be -15*/{
                pivotTelescopeArmMotor.stopMotor();
                pivotTelescopeArmMotorTwo.stopMotor();
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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Extendo pivot limit switch", PivotArmLimitSwitch.get());
        SmartDashboard.putBoolean("Extendo extend limit switch", ExtendLimitSwitch.get());
        SmartDashboard.putNumber("Extendo extend position", extendExtendoMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Extendo pivot position", pivotTelescopeArmMotor.getEncoder().getPosition());

        SmartDashboard.putString("Extendo Pivot Brake mode", pivotTelescopeArmMotor.getIdleMode().toString());
        SmartDashboard.putString("Extendo extend Brake mode", extendExtendoMotor.getIdleMode().toString());
    }
}
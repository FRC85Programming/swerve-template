// package frc.robot.subsystems;
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubystem extends SubsystemBase {
    private final CANSparkMax extendExtendoMotor = new CANSparkMax(Constants.EXTENDO_EXTEND_MOTOR, MotorType.kBrushless);
    private final CANSparkMax pivotTelescopeArmMotor = new CANSparkMax(Constants.EXTENDO_ARM_PIVOT_MOTOR,MotorType.kBrushless);
    private final DigitalInput PivotArmLimitSwitch = new DigitalInput(Constants.EXTENDO_PIVOT_LIMIT_SWITCH);
    private final DigitalInput ExtendLimitSwitch = new DigitalInput(Constants.EXTENDO_EXTEND_LIMIT_SWITCH);
    private final PIDController pid = new PIDController(0, 0, 0);
    private final double extendSpeedScale = 0.2;
    private final double pivotSpeedScale = 0.1;

    public ExtendoSubystem() {
        extendExtendoMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeArmMotor.setIdleMode(IdleMode.kBrake);
    }

    public void ExtendTelescope(double speed) {
        // uses pid controller to get position that is then set to motor
        // extendExtendoMotor.set(pid.calculate(extendExtendoMotor.getEncoder().getPosition(),
        // setPoint));
        // extendExtendoMotor.set(speed);
        if (speed > 0) {
            if (extendExtendoMotor.getEncoder().getPosition() > 55) {
                // stops motor with upper limit switch
                extendExtendoMotor.set(0);
            } else {
                // doesnt stop if limit switch isnt pressed
                extendExtendoMotor.set(speed * extendSpeedScale);
            }
        } else {
            if (ExtendLimitSwitch.get()){
                extendExtendoMotor.getEncoder().setPosition(0);
                extendExtendoMotor.set(0);
            } else {
                extendExtendoMotor.set(speed * extendSpeedScale);
            }
            
        }
    }

    public void Pivot(double speed) {
        // working command for setting pivot speed

        if (speed > 0) {
            if (pivotTelescopeArmMotor.getEncoder().getPosition() > 95) {
                pivotTelescopeArmMotor.set(0);
            } else {
                pivotTelescopeArmMotor.set(speed * pivotSpeedScale);
            }
        } else {
            if (PivotArmLimitSwitch.get()) {
                pivotTelescopeArmMotor.getEncoder().setPosition(0);
                pivotTelescopeArmMotor.set(0);
            } else {
                pivotTelescopeArmMotor.set(speed * pivotSpeedScale);
            }

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
    }
}
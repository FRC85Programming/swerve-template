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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubystem extends SubsystemBase {
    private CANSparkMax extendExtendoMotor = new CANSparkMax(Constants.EXTENDO_EXTEND_MOTOR, MotorType.kBrushless);
    private CANSparkMax pivotTelescopeArmMotor = new CANSparkMax(Constants.EXTENDO_ARM_PIVOT_MOTOR,
            MotorType.kBrushless);
    private CANSparkMax pivotTelescopeIntakemotor = new CANSparkMax(Constants.EXTENDO_INTAKE_PIVOT_MOTOR,
            MotorType.kBrushless);
    private DigitalInput PivotArmLimitSwitch = new DigitalInput(0);
    private PIDController pid = new PIDController(0, 0, 0);

    public ExtendoSubystem() {
        extendExtendoMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeArmMotor.setIdleMode(IdleMode.kBrake);
        pivotTelescopeIntakemotor.setIdleMode(IdleMode.kBrake);
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
                extendExtendoMotor.set(speed * 0.1);
            }
        } else {
            extendExtendoMotor.set(speed * 0.1);
        }
    }

    public void Pivot(double speed) {
        // working command for setting pivot speed

        if (speed > 0) {
            if (pivotTelescopeArmMotor.getEncoder().getPosition() > 95) {
                pivotTelescopeArmMotor.set(0);
            } else {
                pivotTelescopeArmMotor.set(speed * 0.1);
            }
        } else {
            if (PivotArmLimitSwitch.get()){
                pivotTelescopeArmMotor.set(0);
            } else {
                pivotTelescopeArmMotor.set(speed * 0.1);
            }
            
        }
    }
}
// package frc.robot.subsystems;
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubystem extends SubsystemBase {
    private CANSparkMax extendExtendoMotor = new CANSparkMax(Constants.EXTENDO_EXTEND_MOTOR, MotorType.kBrushless);
    private CANSparkMax pivotTelescopeMotor = new CANSparkMax(Constants.EXTENDO_PIVOT_MOTOR, MotorType.kBrushless);
    private PIDController pid = new PIDController(0, 0, 0);

    public void ExtendTelescope(int setPoint){   
        // uses pid controller to get position that is then set to motor
        extendExtendoMotor.set(pid.calculate(extendExtendoMotor.getEncoder().getPosition(), setPoint));
    }

    /**
    * @param position takes in a position setter for location of arm 
    **/ 
    public void PivotTelescope(double position)
    {
        pivotTelescopeMotor.set(pid.calculate(pivotTelescopeMotor.getEncoder().getPosition(), position));
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class IntakeSubsystem extends SubsystemBase {
    public TalonSRX upperIntakeRollerMotor1 = new TalonSRX(Constants.UPPER_INTAKE_ROLLER_MOTOR1);
    public TalonSRX lowerIntakeRollerMotor2 = new TalonSRX(Constants.LOWER_INTAKE_ROLLER_MOTOR2);


public IntakeSubsystem() {
    //Motor setup
    
}

public void setRollerSpeed(double rollerSpeed){
    upperIntakeRollerMotor1.set(ControlMode.PercentOutput, rollerSpeed);
    lowerIntakeRollerMotor2.set(ControlMode.PercentOutput, rollerSpeed);
}
}

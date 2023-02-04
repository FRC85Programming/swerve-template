package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class IntakeSubsystem extends SubsystemBase {
    public TalonSRX upperIntakeRollerMotor1 = new TalonSRX(Constants.UPPER_INTAKE_ROLLER_MOTOR1);
    public VictorSPX lowerIntakeRollerMotor2 = new VictorSPX(Constants.LOWER_INTAKE_ROLLER_MOTOR2);

public void setRollerSpeed(){
    double upperSpeed = SmartDashboard.getNumber("Upper Roller Speed", -1);
    double lowerSpeed = SmartDashboard.getNumber("Lower Roller Speed", -1);
    upperIntakeRollerMotor1.set(ControlMode.PercentOutput, upperSpeed);
    lowerIntakeRollerMotor2.set(ControlMode.PercentOutput, lowerSpeed);
}

public void StopRollers()
{
    upperIntakeRollerMotor1.set(ControlMode.PercentOutput, 0);
    lowerIntakeRollerMotor2.set(ControlMode.PercentOutput, 0);
}
}

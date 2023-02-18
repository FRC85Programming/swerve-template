package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    public CANSparkMax intakeRollerMotor = new CANSparkMax(Constants.INTAKE_ROLLERS_MOTOR, MotorType.kBrushless);

public void setRollerSpeed() {
    double speed = SmartDashboard.getNumber("Roller Speed", 1);
    intakeRollerMotor.set(speed);
}

public void StopRollers()
{
    intakeRollerMotor.set(0);
}
}

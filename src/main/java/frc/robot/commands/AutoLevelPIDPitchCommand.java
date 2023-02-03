package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoLevelPIDPitchCommand extends PIDCommand {

    public static double usfullDrivePower = .30;
    public DrivetrainSubsystem m_DrivetrainSubsystem;

    public AutoLevelPIDPitchCommand(DrivetrainSubsystem drivetrainSubsystem){
        super(
            // The controller that the command will use
        new PIDController(0.045, 0, 0),
        // This should return the measurement
        () -> drivetrainSubsystem.GetPitchRoll()[0],
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
        drivetrainSubsystem.drive(new ChassisSpeeds(-usfullDrivePower, -usfullDrivePower, 0));
        });
    }

}
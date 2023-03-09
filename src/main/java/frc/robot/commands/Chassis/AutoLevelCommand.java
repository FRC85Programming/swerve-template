package frc.robot.commands.Chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoLevelCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public AutoLevelCommand(DrivetrainSubsystem drivetrain) {
        this.m_drivetrainSubsystem = drivetrain;
    }

    @Override
    public void execute() {

        // Code to AutoLevel the Robot Forwards and Backwards if the Robot drives onto
        // the Charging Station Facing Forward
        double[] pr = m_drivetrainSubsystem.GetPitchRoll();
        double p = pr[0];
        double r = pr[1];

        if (p > -5 && p < 5 && r > -5 && r < 5) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        } else {
            double maxSpeed = SmartDashboard.getNumber("AutoLevel Max Speed", .9);
            double constant = SmartDashboard.getNumber("AutoLevel Constant", 0.5);
            double total = Math.abs(p) + Math.abs(r);
            double x = (-constant * Math.abs(r) * r) / total;
            double y = (constant * Math.abs(p) * p) / total;

            if (Math.abs(x) > maxSpeed) {
                x = Math.copySign(maxSpeed, x);
            }

            if (Math.abs(y) > maxSpeed) {
                y = Math.copySign(maxSpeed, y);
            }
            m_drivetrainSubsystem.drive(new ChassisSpeeds(x, y, 0));

        }

    }
    // Code to AutoLevel the Robot Forwards and Backwards if the Robot drives onto
    // the Charging Station Facing The Front Right Module

    @Override

    public boolean isFinished() {
        double[] ypr = m_drivetrainSubsystem.GetPitchRoll();
        return false;

        // Code to stop the Robot from AutoLeveling when Leveled facing forwards
        /*
         * if (ypr[1] < -5 || ypr[1] > 5) {
         * return false;
         * }
         * else
         * {
         * return true;
         * }
         */
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));

    }
}

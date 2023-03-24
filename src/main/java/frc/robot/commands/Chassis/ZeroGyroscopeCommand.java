package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.HomeExtendCommand;
import frc.robot.commands.Arm.TimedIntakeCommand;
import frc.robot.commands.Autos.AutoScore;
import frc.robot.commands.Autos.AutoScoreExtend;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroscopeCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double m_angle;

    public ZeroGyroscopeCommand(DrivetrainSubsystem driveTrain, double angle) {
        this.m_drivetrainSubsystem = driveTrain;
        this.m_angle = angle;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroGyroscope(m_angle);
    }

    @Override

    public boolean isFinished() {
        return true;
    }

}

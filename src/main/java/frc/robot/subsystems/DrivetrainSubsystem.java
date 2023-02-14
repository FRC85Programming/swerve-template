// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 3;

  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final SwerveDriveOdometry odometry;

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  static double[] ypr = new double[3];
  PIDController pitchPIDController = new PIDController(0, 0, 0);
  PIDController rollPIDController = new PIDController(0, 0, 0); 

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
    .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
    .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(0, 0))
    .withSteerOffset(0)
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
    .build();

    // We will do the same for the other modules
    m_frontRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
    .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
    .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(2, 0))
    .withSteerOffset(0)
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
    .build();

    m_backLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
      .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
      .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
      .withSteerOffset(0)
      .withGearRatio(SdsModuleConfigurations.MK4_L2)
      .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
      .build();

    m_backRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
    .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
    .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(6, 0))
    .withSteerOffset(0)
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
    .build();

    odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()), new SwerveModulePosition[]
    {
      m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
    });

    tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
    tab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
    tab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public double pitchOffset;
  public double rollOffset;

  public void zeroGyroscope() {
    //m_pigeon.setYaw(0.0);
    odometry.resetPosition(Rotation2d.fromDegrees(m_pigeon.getFusedHeading()), new SwerveModulePosition[]
    {
      m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
    }, new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)));
  }

  public void zeroPitchRoll()
  {
        pitchOffset = ypr[1];
        rollOffset = ypr[2];
  }

  public double[] GetPitchRoll()
  {
        return new double[] {ypr[1] - pitchOffset, ypr[2] - rollOffset};
  }

//   sets all wheel positions to 45 degrees to prevent movement
  // public void brakeWheels(){

  // m_backLeftModule.set(0, 45);
  // }
  public Rotation2d getGyroscopeRotation() {
    //return Rotation2d.fromDegrees(m_pigeon.getYaw());
    return odometry.getPoseMeters().getRotation();
  }

  public SwerveModule getFrontLeft() {
    return m_backLeftModule;
  }

  public SwerveModule getFrontRight() {
    return m_frontRightModule;
  }

  public SwerveModule getBackRight() {
    return m_backRightModule;
  }

  public SwerveModule getBackLeft() {
    return m_backLeftModule;
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void setChassisSpeeds(double x, double y, double z) {
    drive(m_chassisSpeeds);
  }

  private boolean brakeLock = false;
  //private boolean resetWheels = false;
  private boolean tankLock = false;
  @Override
  public void periodic() {
    if (brakeLock){
      brakeState();
    } else{
      swerveState();
    }

    SmartDashboard.putNumber("Front Left Steer Absolute Angle", Units.radiansToDegrees(m_frontLeftModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("Front Right Steer Absolute Angle", Units.radiansToDegrees(m_frontRightModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("Back Left Steer Absolute Angle", Units.radiansToDegrees(m_backLeftModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("Back Right Steer Absolute Angle", Units.radiansToDegrees(m_backRightModule.getSteerEncoder().getAbsoluteAngle()));
  }

  public void brakeState()
  {
    m_frontLeftModule.set(0, -FRONT_LEFT_MODULE_STEER_OFFSET);
    m_frontRightModule.set(0, -FRONT_RIGHT_MODULE_STEER_OFFSET);
    m_backLeftModule.set(0, -BACK_LEFT_MODULE_STEER_OFFSET);
    m_backRightModule.set(0, -BACK_RIGHT_MODULE_STEER_OFFSET);
  }


  public void tankState(RobotContainer controller)
  {
    m_frontLeftModule.set(controller.getController().getLeftY(), 0);
    m_frontRightModule.set(controller.getController().getLeftX(), 0);
    m_backLeftModule.set(controller.getController().getLeftY(), 0);
    m_backRightModule.set(controller.getController().getLeftX(), 0);
  }

  public void swerveState()
  {
    odometry.update(Rotation2d.fromDegrees(m_pigeon.getFusedHeading()), new SwerveModulePosition[]
    {
      m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
    });

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians() - FRONT_LEFT_MODULE_STEER_OFFSET);
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians() - FRONT_RIGHT_MODULE_STEER_OFFSET);
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians() - BACK_LEFT_MODULE_STEER_OFFSET);
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians() - BACK_RIGHT_MODULE_STEER_OFFSET);
       
    m_pigeon.getYawPitchRoll(ypr);
    double[] PitchRoll = GetPitchRoll();
    SmartDashboard.putNumber("Gyro Yaw", ypr[0]);
    SmartDashboard.putNumber("Gyro Pitch", PitchRoll[0]);
    SmartDashboard.putNumber("Gyro Roll", PitchRoll[1]);
  }

  public void AutoMode(){
    
  }

  // sets true or false for brake command  
  public void setLock(boolean value){
    brakeLock = value;
  }


  public void AutoLevelPIDController()
  {

    double kp = SmartDashboard.getNumber("kp", 0);
    double ki = SmartDashboard.getNumber("ki", 0);
    double kd = SmartDashboard.getNumber("kd", 0);

    double[] pr = GetPitchRoll();
    double maxSpeed = 1;


    pitchPIDController.setPID(kp, ki, kd);
    rollPIDController.setPID(kp, ki, kd);

    double x = pitchPIDController.calculate(pr[0],0);
    double y = rollPIDController.calculate(pr[1],0);

      if(Math.abs(x) > maxSpeed)
      {
          x = Math.copySign(maxSpeed, x);
      }

      if(Math.abs(y) > maxSpeed)
      {
          y = Math.copySign(maxSpeed, y);
      }

      SmartDashboard.putNumber("AutoLevelPID x", x);
      SmartDashboard.putNumber("AutoLevelPID y", y);

      if (pr[0] > -5 && pr[0] < 5 ) 
      {
        x = 0;
      }

      if (pr[1] > -5 && pr[1] < 5) 
      {
        y = 0;
      }
      drive(new ChassisSpeeds(y,-x,0));
  }

}

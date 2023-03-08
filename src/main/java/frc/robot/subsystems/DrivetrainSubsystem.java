// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12;

  private NetworkTable _calibration = NetworkTableInstance.getDefault().getTable("SwerveCalibration");
  private NetworkTableEntry _frontLeftCalibration;
  private NetworkTableEntry _frontRightCalibration;
  private NetworkTableEntry _backLeftCalibration;
  private NetworkTableEntry _backRightCalibration;
  private double _frontLeftCalibrationValue;
  private double _frontRightCalibrationValue;
  private double _backLeftCalibrationValue;
  private double _backRightCalibrationValue;

  


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
        SdsModuleConfigurations.MK4_L1.getDriveReduction() * 
        SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
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
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
  static double[] ypr = new double[3];
  PIDController pitchPIDController = new PIDController(0, 0, 0);
  PIDController rollPIDController = new PIDController(0, 0, 0); 

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final PIDController turningPidController;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    _frontLeftCalibration = _calibration.getEntry("FrontLeft");
    _frontRightCalibration = _calibration.getEntry("FrontRight");
    _backLeftCalibration = _calibration.getEntry("BackLeft");
    _backRightCalibration = _calibration.getEntry("BackRight");
    _frontLeftCalibration.setPersistent();
    _frontRightCalibration.setPersistent();
    _backLeftCalibration.setPersistent();
    _backRightCalibration.setPersistent();
    _frontLeftCalibrationValue = _frontLeftCalibration.getDouble(FRONT_LEFT_MODULE_STEER_OFFSET);
    _frontRightCalibrationValue = _frontRightCalibration.getDouble(FRONT_RIGHT_MODULE_STEER_OFFSET);
    _backLeftCalibrationValue = _backLeftCalibration.getDouble(BACK_LEFT_MODULE_STEER_OFFSET);
    _backRightCalibrationValue = _backRightCalibration.getDouble(BACK_RIGHT_MODULE_STEER_OFFSET);


    SmartDashboard.putBoolean("Swerve Calibrate", false); 

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

    odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), new SwerveModulePosition[]
    {
      m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
    });

    turningPidController = new PIDController(Constants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
    tab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
    tab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    
    SmartDashboard.putNumber("SwerveDrive P", getDrivePID().getP());
    SmartDashboard.putNumber("SwerveDrive I", getDrivePID().getI());
    SmartDashboard.putNumber("SwerveDrive D", getDrivePID().getD());
    SmartDashboard.putBoolean("Set drive PID", false);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public double pitchOffset;
  public double rollOffset;
  

  public void zeroGyroscope() {
    m_pigeon.setYaw(0.0);
    odometry.resetPosition(Rotation2d.fromDegrees(m_pigeon.getCompassHeading()), new SwerveModulePosition[]
    {
      m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
    }, new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)));
  }

  // public void resetOdometry() {
  //   odometry.resetPosition(Rotation2d.fromDegrees(m_pigeon.getFusedHeading()), new SwerveModulePosition[]
  //   {
  //     m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
  //   }, new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)));
  // }

  public void zeroPitchRoll()
  {
        pitchOffset = ypr[1];
        rollOffset = ypr[2];
  }

  public double[] GetPitchRoll()
  {
        return new double[] {ypr[1] - pitchOffset, ypr[2] - rollOffset};
  }

  public SwerveModule getFrontLeft() {
    return m_frontLeftModule;
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


//   sets all wheel positions to 45 degrees to prevent movement
  // public void brakeWheels(){

  // m_backLeftModule.set(0, 45);
  // }
  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }
// this sucks -matt
  public void setChassisSpeeds(double x, double y, double z) {
    drive(m_chassisSpeeds);
  }

  public double getHeading() {
    return Math.IEEEremainder(m_pigeon.getCompassHeading(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    // Gets the position of the bot on the field for the auto
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    // Resets the gyro
    SwerveModulePosition positions[] = {m_backLeftModule.getPosition(), m_backRightModule.getPosition(), m_frontLeftModule.getPosition(), m_frontRightModule.getPosition()};
    odometry.resetPosition(getRotation2d(), positions, pose);
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Sets the desired states of the modules (this is where the error is) 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
    setDesiredState(desiredStates[0], m_frontLeftModule);
    setDesiredState(desiredStates[1], m_frontRightModule);
    setDesiredState(desiredStates[2], m_backLeftModule);
    setDesiredState(desiredStates[3], m_backRightModule);
  }

  public void setDesiredState(SwerveModuleState state, SwerveModule module) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState(module).angle);
    module.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond, turningPidController.calculate(getTurningPosition(module), state.angle.getRadians()));
  }


  public SwerveModuleState getState(SwerveModule module) {
    return new SwerveModuleState(getDriveVelocity(module), new Rotation2d(getTurningPosition(module)));
  }

  public double getTurningPosition(SwerveModule module) {
    return module.getSteerAngle();
  }

  public double getTurningVelocity(SwerveModule module) {
    return module.getSteerMotor().get();
  }

  public double getDriveVelocity(SwerveModule module) {
    return module.getDriveVelocity();
  }

  private boolean brakeLock = false;
  private boolean halfSpeedLock = false;
  @Override
  public void periodic() {
    SwerveModulePosition positions[] = {m_backLeftModule.getPosition(), m_backRightModule.getPosition(), m_frontLeftModule.getPosition(), m_frontRightModule.getPosition()};
    odometry.update(getRotation2d(), positions);
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    if (brakeLock){
      brakeState();
    }
    else if (halfSpeedLock == true) {
      swerveState(true);
    } else {
      swerveState(false);
    }

    SmartDashboard.putNumber("Speeds", m_backLeftModule.getDriveDistance());

    SmartDashboard.putNumber("Front Left Steer Absolute Angle", Units.radiansToDegrees(m_frontLeftModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("Front Right Steer Absolute Angle", Units.radiansToDegrees(m_frontRightModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("Back Left Steer Absolute Angle", Units.radiansToDegrees(m_backLeftModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("Back Right Steer Absolute Angle", Units.radiansToDegrees(m_backRightModule.getSteerEncoder().getAbsoluteAngle()));

    SmartDashboard.putNumber("Front Left Steer Calibration Angle", Units.radiansToDegrees(_frontLeftCalibrationValue));
    SmartDashboard.putNumber("Front Right Steer Calibration Angle", Units.radiansToDegrees(_frontRightCalibrationValue));
    SmartDashboard.putNumber("Back Left Steer Calibration Angle", Units.radiansToDegrees(_backLeftCalibrationValue));
    SmartDashboard.putNumber("Back Right Steer Calibration Angle", Units.radiansToDegrees(_backRightCalibrationValue));

    if (SmartDashboard.getBoolean("Set drive PID", false)) {
      double p = SmartDashboard.getNumber("SwerveDrive P", getDrivePID().getP());
      double i = SmartDashboard.getNumber("SwerveDrive I", getDrivePID().getI());
      double d = SmartDashboard.getNumber("SwerveDrive D", getDrivePID().getD());
      setDrivePID(p, i, d);
      SmartDashboard.putBoolean("Set drive PID", false);
    }
  }

  public void checkCalibration() {
    boolean calibrate = SmartDashboard.getBoolean("Swerve Calibrate", false);
    if (calibrate) {
      _frontLeftCalibrationValue = -m_frontLeftModule.getSteerEncoder().getAbsoluteAngle(); 
      _frontRightCalibrationValue = -m_frontRightModule.getSteerEncoder().getAbsoluteAngle();
      _backLeftCalibrationValue = -m_backLeftModule.getSteerEncoder().getAbsoluteAngle();
      _backRightCalibrationValue = -m_backRightModule.getSteerEncoder().getAbsoluteAngle();
      _frontLeftCalibration.setDouble(_frontLeftCalibrationValue);
      _frontRightCalibration.setDouble(_frontRightCalibrationValue);
      _backLeftCalibration.setDouble(_backLeftCalibrationValue);
      _backRightCalibration.setDouble(_backRightCalibrationValue);
      _frontLeftCalibration.setPersistent();
      _frontRightCalibration.setPersistent();
      _backLeftCalibration.setPersistent();
      _backRightCalibration.setPersistent();
      SmartDashboard.putBoolean("Swerve Calibrate", false);
    }
  }
  public double getFrontLeftCalibration() {
    return _frontLeftCalibrationValue;
  }
  public double getBackLeftCalibration() {
    return _backLeftCalibrationValue;
  }
  public double getFrontRightCalibration() {
    return _frontRightCalibrationValue;
  }
  public double getBackRightCalibration() {
    return _backRightCalibrationValue;
  }

  public void swerveDriveVolts(double frontLeft, double frontRight, double backRight, double backLeft) {
    m_frontLeftModule.set(frontLeft, 0 - -_frontLeftCalibrationValue);
    m_frontRightModule.set(frontRight, 0 - -_frontRightCalibrationValue);
    m_backLeftModule.set(backLeft, 0 - -_backLeftCalibrationValue);
    m_backRightModule.set(backRight, 0 - -_backRightCalibrationValue);
  }
  public void brakeState()
  {
    m_frontLeftModule.set(0, 45 - -_frontLeftCalibrationValue);
    m_frontRightModule.set(0, 45 - -_frontRightCalibrationValue);
    m_backLeftModule.set(0, 45 - -_backLeftCalibrationValue);
    m_backRightModule.set(0, 45 - -_backRightCalibrationValue);
  }

  public void swerveState(boolean halfSpeed)
  {
    // odometry.update(getGyroscopeRotation(), new SwerveModulePosition[]
    // {
    //   m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()
    // });
    double scale = 1;
    if (halfSpeed){
      scale = 0.5;
    }
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond * scale / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians() - _frontLeftCalibrationValue);
    m_frontRightModule.set(states[1].speedMetersPerSecond * scale / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians() - _frontRightCalibrationValue);
    m_backLeftModule.set(states[2].speedMetersPerSecond * scale / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians() - _backLeftCalibrationValue);
    m_backRightModule.set(states[3].speedMetersPerSecond * scale / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians() - _backRightCalibrationValue);
       
    m_pigeon.getYawPitchRoll(ypr);
    double[] PitchRoll = GetPitchRoll();
    SmartDashboard.putNumber("Gyro Yaw", ypr[0]);
    SmartDashboard.putNumber("Gyro Pitch", PitchRoll[0]);
    SmartDashboard.putNumber("Gyro Roll", PitchRoll[1]);
  }

  
  // sets true or false for brake command  
  public void setLock(boolean value){
    brakeLock = value;
  }
  public void setHalfSpeed(boolean value)
  {
    halfSpeedLock = value;
  }


  public void AutoLevelPIDController()
  {

    double kp = SmartDashboard.getNumber("kp", 0);
    double ki = SmartDashboard.getNumber("ki", 0);
    double kd = SmartDashboard.getNumber("kd", 0);

    double[] pr = GetPitchRoll();
    double maxSpeed = .6;


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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_frontLeftModule.getDriveVelocity(), m_backRightModule.getDriveVelocity());
  }
    
  public void stop() {
    drive(new ChassisSpeeds(0.0,0.0,0.0));
  }

  public SparkMaxPIDController getDrivePID(){
    return ((CANSparkMax)m_frontLeftModule.getDriveMotor()).getPIDController();
  }

  public void setDrivePID(double p, double i, double d){
    CANSparkMax frontLeft = (CANSparkMax)m_frontLeftModule.getDriveMotor();
    frontLeft.getPIDController().setP(p);
    frontLeft.getPIDController().setI(i);
    frontLeft.getPIDController().setD(d);

    CANSparkMax frontRight = (CANSparkMax)m_frontRightModule.getDriveMotor();
    frontRight.getPIDController().setP(p);
    frontRight.getPIDController().setI(i);
    frontRight.getPIDController().setD(d);
    
    CANSparkMax backRight = (CANSparkMax)m_backRightModule.getDriveMotor();
    backRight.getPIDController().setP(p);
    backRight.getPIDController().setI(i);
    backRight.getPIDController().setD(d);

    CANSparkMax backLeft = (CANSparkMax)m_backLeftModule.getDriveMotor();
    backLeft.getPIDController().setP(p);
    backLeft.getPIDController().setI(i);
    backLeft.getPIDController().setD(d);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.ml.StatModel;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .4953; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .4953; 

    public static final int DRIVETRAIN_PIGEON_ID = 41;

    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(252);

    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(130);

    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 21; 
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 22; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(319);

    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 31;
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 32; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 33; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(210.5);

    public static final int EXTENDO_EXTEND_MOTOR = 51;
    public static final int EXTENDO_ARM_PIVOT_MOTOR = 52;
    public static final int PIVOT_LOCK_SERVO = 0;

    public static final int INTAKE_ROLLERS_MOTOR = 54;
    public static final int INTAKE_PIVOT_MOTOR = 53;

    public static final int INTAKE_PIVOT_LIMIT_SWITCH = 2;
    public static final int EXTENDO_EXTEND_LIMIT_SWITCH = 1;
    public static final int EXTENDO_PIVOT_LIMIT_SWITCH = 0;

}

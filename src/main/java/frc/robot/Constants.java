// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <>It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
        public static final int PDP_ID = 58; // PDP Id

        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;

        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -3.799670;

        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -2.722816;

        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 21;
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 22;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -3.187612;

        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 31;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 32;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 33;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -3.574175;

        public static final int EXTEND_MOTOR = 51;
        public static final int PIVOT_MOTOR = 52;
        public static final int PIVOT_MOTOR_TWO = 55;

        public static final int INTAKE_ROLLERS_MOTOR = 54;
        public static final int INTAKE_PIVOT_MOTOR = 53;

        public static final int WRIST_LIMIT_SWITCH = 2;
        public static final int EXTEND_LIMIT_SWITCH = 1;
        public static final int PIVOT_LIMIT_SWITCH = 0;

        // Path modules constants
        public static final double kPTurning = 0.5;



}

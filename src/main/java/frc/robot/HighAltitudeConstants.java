// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.chassis.DriveTrain.TransmissionMode;
import frc.robot.subsystems.chassis.DriveTrain.WheelState;
import frc.robot.subsystems.intake.Intake.IntakePosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class HighAltitudeConstants {

        public static final boolean DEBUG = false;

        public static final boolean SINGLE_DRIVER = false;

        public static final boolean USING_VISION_FOR_POSE = true;

        ////////////////////////// SWERVE //////////////////////////

        /////////// KINEMATICS
        // Distance left - right
        public static final double SWERVE_TRACK_WIDTH = 0.0254 * (24.0 - 2.0 * 2.625);
        // Distance front - back
        public static final double SWERVE_WHEEL_BASE = 0.0254 * (32.0 - 2.0 * 2.625);

        // FL, FR, BL, BR. Remember these cartesian coordinates consider the x axis to
        // be headed where the robot is pointing to. The y-axis direction could be a
        // source of problems...
        // WPILib says "Positive x values represent moving toward the front of the robot
        // whereas positive y values represent moving toward the left of the robot."
        // The example I saw uses the raw yaw reported by the navx and switches the
        // position of the left and right wheels in the kinematics.
        // I will use CCW and the allegedly correct x y coordinates.
        // For some reason, that did not work. The kinematics seem to work correctly
        // when "left" is negative
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(SWERVE_WHEEL_BASE / 2, SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(SWERVE_WHEEL_BASE / 2, -SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(-SWERVE_WHEEL_BASE / 2, SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(-SWERVE_WHEEL_BASE / 2, -SWERVE_TRACK_WIDTH / 2));

        // Arbitrary. Higher numbers will cause the swerve to react more violently to
        // joysitck inputs and may not be ideal. Lower numbers will cause the swerve to
        // have a very slow reaction to joystick inputs, and may not be ideal.
        public static final double SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND = 4.0;
        public static final double SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 4.0;

        // Other

        public static final double SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION = 4096.0;
        // encoder * this value = radians
        public static final double SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE = (2.0 * Math.PI)
                        / SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION;

        /////////// DRIVING MOTOR

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DRIVE_PULSES_PER_REVOLUTION = 2048.0;
        public static final double SWERVE_DRIVE_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // In meters
        public static final double SWERVE_WHEEL_DIAMETER = 4 * 0.0254;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver)
        // Constant for L3 Configuration
        public static final double SWERVE_DRIVE_GEAR_RATIO = (50.0 * 16.0 * 45.0) / (14.0 * 28.0 * 15.0);

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_METERS_PER_PULSE = Math.PI * SWERVE_WHEEL_DIAMETER
                        / (SWERVE_DRIVE_PULSES_PER_REVOLUTION * SWERVE_DRIVE_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = meters/second
        public static final double SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS = (1000
                        * SWERVE_DRIVE_METERS_PER_PULSE)
                        / SWERVE_DRIVE_VELOCITY_SAMPLE_RATE_MS;

        // Constant for L3 Configuration
        public static final double SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND = 18 * 12 * 0.0254;

        // Arbitrary to make controlling the swerve easier in teleop
        public static final double SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND = SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
                        / 2;

        /////////// DIRECTION MOTOR

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DIRECTION_PULSES_PER_REVOLUTION = 2048.0;
        public static final double SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver)
        public static final double SWERVE_DIRECTION_GEAR_RATIO = 150.0 / 7.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = radians
        public static final double SWERVE_DIRECTION_RADIANS_PER_PULSE = Math.PI * 2
                        / (SWERVE_DIRECTION_PULSES_PER_REVOLUTION * SWERVE_DIRECTION_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = radians/second
        public static final double SWERVE_DIRECTION_RADIANS_PER_SEC_PER_VELOCITY_UNITS = (1000
                        * SWERVE_DIRECTION_RADIANS_PER_PULSE)
                        / SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS;

        public static final double SWERVE_DIRECTION_TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

        //// PID
        public static final double SWERVE_DIRECTION_BRAKING_RADIANS = (Math.PI * 2) / 4; // 2pi/3
        public static final double SWERVE_DIRECTION_KP = 0.8;
        public static final double SWERVE_DIRECTION_KD = 0.4;
        //////////////////////// DRIVETRAIN///////////////////////////////////

        //// DRAGONFLY

        // The constant that helps the robot stay in the same angle while driving on an
        // assisted mode with the dragonfly. It helps correct the current error in the
        // angle.
        // The higher this constant, the more it will turn to correct, specially after
        // braking.
        public static final double DRIVETRAIN_DRAGONFLY_ANGLE_CORRECTION = 0.01;
        // This constants also helps the robot stay in the same angle while driving on
        // an
        // assited mode with the dragonfly. It helps predict the error that will be
        // generated
        // by the dragonfly and corrects based on that. The higher the constant, the
        // more it will
        // turn while driving.
        public static final double DRIVETRAIN_DRAGONFLY_EXPECTED_CORRECTION = 0.5;

        // The sides of the robot normally move faster than the dragonfly module, this
        // constant indicates their rate. A constant of 0.60 means that the dragonfly
        // module has 60% of the speed of the sides.
        public static double DRIVETRAIN_DRAGONFLY_SIDES_CORRECTION = 0.6;

        //// DEFAULT INITIAL PARAMETERS

        public static TransmissionMode DRIVETRAIN_INITIAL_TRANSMISSION_MODE = TransmissionMode.torque;
        public static WheelState DRIVETRAIN_INITIAL_DRAGONFLY_STATE = WheelState.Raised;

        // Default braking mode, true for brake, false for coast.
        public static boolean DRIVETRAIN_MOTORS_BRAKING_MODE = true;

        ///// AUTOS

        /// Turning

        // When turning in autonomous at power 1, if the difference between the target
        // angle and the current angle (in degrees) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // turning speed.
        public static double DRIVETRAIN_AUTO_TURNING_BRAKING_DISTANCE = 100; // 100
        // When turning in autonomous, if the difference between the target angle and
        // the current angle (in degrees) is less than this constant, it will be
        // considered on target.
        public static double DRIVETRAIN_AUTO_TURNING_ARRIVE_OFFSET = 3; // 3

        /// Straight motion

        // When moving straight (straightMove()) in autonomous at power 1, if the
        // difference between the target and the current position (in meters) is less
        // than this constant, it will start braking. Note that this constant is
        // proportional to the square of the speed (from -1 to 1).
        public static double DRIVETRAIN_AUTO_STRAIGHT_BRAKING_DISTANCE = 1;
        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in meters) is less than this
        // constant, it will be considered on target.
        public static double DRIVETRAIN_AUTO_STRAIGHT_ARRIVE_OFFSET = 0.05;
        // When moving straight while keeping an angle (straightMove()), this constant
        // will determine how sharp the angle correction is. The higher the values, the
        // sharper the angle correction.
        public static double DRIVETRAIN_AUTO_STRAIGHT_ANGLE_CORRECTION = 0.01;

        /// Splines

        // When following a spline, this constant will help correct the error in y.
        public static final double SPLINE_DRIVE_ERROR_CORRECTION = 70; // 70
        // When following a spline, if the difference between the target
        // and the current position (in meters) is less than this constant, it be
        // consider on target.
        public static final double DRIVETRAIN_SPLINE_ARRIVE_OFFSET = 0.05; // 0
        // When following a spline, this constant will determine
        // how sharp the angle correction is. The higher the values, the sharper the
        // angle correction.
        public static final double DRIVETRAIN_SPLINE_ANGLE_CORRECTION = 0.0085; // 0
        public static final double SPLINE_SPEED_REDUCTION_BRAKING_DISTANCE = 0.25; // 0

        /// Alignment
        public static final double DRIVETRAIN_ALIGN_MAX_SPEED = 0.1875;

        /// Auto balancing

        // Units are in degrees and seconds

        // If the absolute value of the angular acceleration is smaller than this value,
        // The robot will be consider as stable on the charging station
        public static final double BALANCING_ACCELERATION_FWD_THRESHOLD = 200.0;

        // If the absolute value of the angular acceleration is smaller than this value,
        // The robot will be consider as stable on the charging station
        public static final double BALANCING_SPEED_THRESHOLD = 18.0;

        // If the absolute value of the angle (pitch) is smaller than this value,
        // The robot will be consider as balanced on the charging station.
        // Be careful with small values (less than about 10°), because some bounciness
        // can occur.
        public static final double BALANCING_ANGLE_THRESHOLD = 10.0;

        // When trying to balance, this is the default power at which the robot will
        // move to
        // Try to balance the charging station. It is recommended to set it low to
        // improve accuracy.
        public static final double BALANCING_DEFAULT_POWER = 0.1875;

        // To prevent the robot from falling off the charging station, it will not move
        // Unless it's properly aligned, that is to say, its angle is less than this
        // value.
        public static final double BALANCING_ALIGNED_THRESHOLD = 5;

        ///// ENCODERS AND GEARBOX

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double DRIVETRAIN_PULSES_PER_REVOLUTION = 2048.0;

        // In meters
        public static final double DRIVETRAIN_WHEEL_DIAMETER = 4 * 0.0254;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS.
        public static final double DRIVETRAIN_GEAR_RATIO_SPEED = 100.0 / 21.0;
        public static final double DRIVETRAIN_GEAR_RATIO_TORQUE = 500.0 / 49.0;

        // Use these constants to convert from encoder position to meters
        // (position*these constants = meters)
        public static final double DRIVETRAIN_METERS_PER_PULSE_TORQUE = 2.1428 * Math.PI * DRIVETRAIN_WHEEL_DIAMETER
                        / (DRIVETRAIN_PULSES_PER_REVOLUTION * DRIVETRAIN_GEAR_RATIO_TORQUE);
        public static final double DRIVETRAIN_METERS_PER_PULSE_SPEED = Math.PI * DRIVETRAIN_WHEEL_DIAMETER
                        / (DRIVETRAIN_PULSES_PER_REVOLUTION * DRIVETRAIN_GEAR_RATIO_SPEED);

        ///////////////////////////// WRIST ////////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean WRIST_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double WRIST_PULSES_PER_REVOLUTION = 2048;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / WRIST REVS
        public static final double WRIST_RATIO = (4959.0 / 49.0);

        public static final double WRIST_DEGREES_PER_PULSE = 360 / (WRIST_PULSES_PER_REVOLUTION * WRIST_RATIO);

        // When moving to a position at power 1, if the difference between the target
        // and the current position (in degrees) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // speed (from -1 to 1).
        public static final double WRIST_BRAKING_DEGREES = 12.5; // 15

        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in meters) is less than this
        // constant, it will be considered on target.
        public static final double WRIST_ARRIVE_OFFSET = 5.0;

        public static final double WRIST_UPPER_LIMIT_DEGREES = 999999999; // -43.34
        public static final double WRIST_LOWER_LIMIT_DEGREES = -999999999; // 111.36

        public static final double WRIST_ARM_DELTA_UPPER_LIMIT = 153.0;
        public static final double WRIST_ARM_DELTA_LOWER_LIMIT = -31.0;

        ///////////////////////////// EXTENSOR /////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean EXTENSOR_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double EXTENSOR_PULSES_PER_REVOLUTION = 2048.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / ARM REVS
        public static final double EXTENSOR_RATIO = 42.0 / 1.0;

        public static final double EXTENSOR_PITCH_DIAMETER_METERS = 1.432 * 0.0254;

        public static final double EXTENSOR_METERS_PER_PULSE = (Math.PI * EXTENSOR_PITCH_DIAMETER_METERS)
                        / (EXTENSOR_PULSES_PER_REVOLUTION * EXTENSOR_RATIO);

        // When moving to a position at power 1, if the difference between the target
        // and the current position (in meters) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // speed (from -1 to 1).
        public static final double EXTENSOR_BRAKING_METERS = 0.05;
        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in meters) is less than this
        // constant, it will be considered on target.

        public static final double EXTENSOR_ARRIVE_OFFSET = 0.025;

        public static final double EXTENSOR_UPPER_LIMIT_METERS = 0.566; // 0.475
        public static final double EXTENSOR_LOWER_LIMIT_METERS = 0.02; // -0.06

        //////////////////////////////// ARM ///////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean ARM_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double ARM_PULSES_PER_REVOLUTION = 2048.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / ARM REVS
        public static final double ARM_RATIO = (94221.0 / 343.0) / 1.0895; // El 1.0895 no sé de dónde salió pero
                                                                           // funciona bien

        public static final double ARM_DEGREES_PER_PULSE = 360 / (ARM_PULSES_PER_REVOLUTION * ARM_RATIO);

        // When moving to a position at power 1, if the difference between the target
        // and the current position (in degrees) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // speed (from -1 to 1).
        public static final double ARM_BRAKING_DEGREES = 180.0; // PREVIOUSLY 18
        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in degrees) is less than this
        // constant, it will be considered on target.

        public static final double ARM_ARRIVE_OFFSET = 5.0;

        public static final double ARM_UPPER_LIMIT_DEGREES = 999999999;
        public static final double ARM_LOWER_LIMIT_DEGREES = -999999999;

        // ARM ODOMETRY

        // Imagine a cartesian plane with the x-axis being located on the floor,
        // increasing towards the front of the robot and the y-axis increasing towards
        // the ceiling. The origin is located at the edge of frame perimeter at the
        // front of the robot.
        public static final Translation2d ARM_CARRIAGE_ZERO_TRANSLATION2D_METERS = new Translation2d(-0.11, 0.31);
        public static final Translation2d ARM_CARRIAGE_BOTTOM_TO_PIVOT_TRANSLATION2D_METERS = new Translation2d(-0.39,
                        0.13).rotateBy(Rotation2d.fromDegrees(-35));
        public static final Translation2d ARM_ROD_TRANSLATION2D_METERS = new Translation2d(0.981, 0);
        public static final Double ARM_INTIAL_ANGLE = 110.0;

        //////////////////////// TRANSPORT CONSTANTS ///////////////////////////////////

        ////////////////////// AUTONOMOUS MOVEMENT

        public static final double WRIST_AUTO_MAX_POWER = 0.25;
        public static final double EXTENSOR_AUTO_MAX_POWER = 0.875;
        public static final double ARM_AUTO_MAX_POWER = 0.5;

        /////////////////// CONE MODE ///////////////////

        ////////// TOP ROW - BACK

        public static final double WRIST_TOP_ROW_BACK_DEGREES_CONE = 0.0;
        public static final double EXTENSOR_TOP_ROW_BACK_METERS_CONE = 0.567;
        public static final double ARM_TOP_ROW_BACK_DEGREES_CONE = 35.28;

        ////////// MIDDLE ROW - BACK

        public static final double WRIST_MIDDLE_ROW_BACK_DEGREES_CONE = 31.74;
        public static final double EXTENSOR_MIDDLE_ROW_BACK_METERS_CONE = 0.31;
        public static final double ARM_MIDDLE_ROW_BACK_DEGREES_CONE = 29.58;

        ////////// TOP ROW - FRONT

        public static final double WRIST_TOP_ROW_FRONT_DEGREES_CONE = 0.0;
        public static final double EXTENSOR_TOP_ROW_FRONT_METERS_CONE = 0.0;
        public static final double ARM_TOP_ROW_FRONT_DEGREES_CONE = 0.0;

        ////////// MIDDLE ROW - FRONT

        public static final double WRIST_MIDDLE_ROW_FRONT_DEGREES_CONE = -172.06; // -197.44
        public static final double EXTENSOR_MIDDLE_ROW_FRONT_METERS_CONE = 0.018; // 0.018
        public static final double ARM_MIDDLE_ROW_FRONT_DEGREES_CONE = -83.04; // -77.14

        ////////// BOTTOM ROW
        /*
         * public static final double WRIST_BOTTOM_ROW_DEGREES_CONE = -1.0;
         * public static final double EXTENSOR_BOTTOM_ROW_METERS_CONE = -1.0;
         * public static final double ARM_BOTTOM_ROW_DEGREES_CONE = -1.0;
         */

        ////////// GRAB FROM FEEDER

        public static final double WRIST_FEEDER_DEGREES_CONE = -187.28; // -142.9 //-142.9
        public static final double EXTENSOR_FEEDER_METERS_CONE = 0.209; // 0.52 //0.01
        public static final double ARM_FEEDER_DEGREES_CONE = -75.8; // -159.02 //-59.39

        ////////// GRAB FROM INTAKE
        public static final double WRIST_INTAKE_DEGREES_CONE = -247.13;
        public static final double EXTENSOR_INTAKE_METERS_CONE = 0.289;
        public static final double ARM_INTAKE_DEGREES_CONE = -137.05;

        //////////// POSITION TO GRAB FROM FLOOR
        public static final double WRIST_FLOOR_DEGREES_CONE = -251.27;
        public static final double EXTENSOR_FLOOR_METERS_CONE = 0.289;
        public static final double ARM_FLOOR_DEGREES_CONE = -152.17;

        ///////// RESTING
        public static final double WRIST_REST_DEGREES_CONE = 0.0; // -274.65
        public static final double EXTENSOR_REST_METERS_CONE = 0.0; // 0.43
        public static final double ARM_REST_DEGREES_CONE = 0.0; // -140.35

        /////////////////// CUBE MODE ///////////////////

        ////////// TOP ROW - BACK

        public static final double WRIST_TOP_ROW_BACK_DEGREES_CUBE = 56.7;
        public static final double EXTENSOR_TOP_ROW_BACK_METERS_CUBE = 0.568;
        public static final double ARM_TOP_ROW_BACK_DEGREES_CUBE = 43.8;

        ////////// MIDDLE ROW - BACK
        public static final double WRIST_MIDDLE_ROW_BACK_DEGREES_CUBE = 70.36;
        public static final double EXTENSOR_MIDDLE_ROW_BACK_METERS_CUBE = 0.29;
        public static final double ARM_MIDDLE_ROW_BACK_DEGREES_CUBE = 50.6;

        ////////// TOP ROW - FRONT

        public static final double WRIST_TOP_ROW_FRONT_DEGREES_CUBE = 0.0;
        public static final double EXTENSOR_TOP_ROW_FRONT_METERS_CUBE = 0.0;
        public static final double ARM_TOP_ROW_FRONT_DEGREES_CUBE = 0.0;

        ////////// MIDDLE ROW - FRONT

        public static final double WRIST_MIDDLE_ROW_FRONT_DEGREES_CUBE = -109.87;
        public static final double EXTENSOR_MIDDLE_ROW_FRONT_METERS_CUBE = 0.017;
        public static final double ARM_MIDDLE_ROW_FRONT_DEGREES_CUBE = -90.07;

        ////////// BOTTOM ROW
        /*
         * public static final double WRIST_BOTTOM_ROW_DEGREES_CUBE = -1.0;
         * public static final double EXTENSOR_BOTTOM_ROW_METERS_CUBE = -1.0;
         * public static final double ARM_BOTTOM_ROW_DEGREES_CUBE = -1.0;
         */
        ////////// GRAB FROM FEEDER

        public static final double WRIST_FEEDER_DEGREES_CUBE = -146.16;
        public static final double EXTENSOR_FEEDER_METERS_CUBE = 0.018;
        public static final double ARM_FEEDER_DEGREES_CUBE = -57.22;

        ////////// GRAB FROM INTAKE

        public static final double WRIST_INTAKE_DEGREES_CUBE = -148.6;
        public static final double EXTENSOR_INTAKE_METERS_CUBE = 0.011;
        public static final double ARM_INTAKE_DEGREES_CUBE = -148.4;

        ///////// POSITION TO GRAB FROM FLOOR

        public static final double WRIST_FLOOR_DEGREES_CUBE = -148.9;
        public static final double EXTENSOR_FLOOR_METERS_CUBE = 0.011;
        public static final double ARM_FLOOR_DEGREES_CUBE = -158.97;

        ///////// RESTING
        public static final double WRIST_REST_DEGREES_CUBE = 0.0; // -214.2
        public static final double EXTENSOR_REST_METERS_CUBE = 0.0; // 0.43
        public static final double ARM_REST_DEGREES_CUBE = 0.0; // -155.22

        ///////////////////////////// GRIPPER //////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean GRIPPER_MOTORS_BRAKING_MODE = true;

        public static final double GRIPPER_CUBE_IN_SPEED = 0.3;
        public static final double GRIPPER_CUBE_OUT_SPEED = -1;
        public static final double GRIPPER_CUBE_HOLD_SPEED = 0.09;
        public static final double GRIPPER_CONE_IN_SPEED = -0.3;
        public static final double GRIPPER_CONE_OUT_SPEED = 1;
        public static final double GRIPPER_CONE_HOLD_SPEED = -0.063;

        public static final double GRIPPER_DEFAULT_IN_SPEED = -0.5;
        public static final double GRIPPER_DEFAULT_OUT_SPEED = 0.5;

        ///////////////////////////// INTAKE //////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean INTAKE_MOTORS_BRAKING_MODE = true;

        public static final IntakePosition INTAKE_INITIAL_POSITION = IntakePosition.STORED;

        public static final double INTAKE_IN_SPEED = 0.5;
        public static final double INTAKE_OUT_SPEED = -0.5;

        ///////////////////// LEDS ////////////////////////

        public static final int LEDS_9280_HUE = 64;
        public static final int LEDS_CUBE_HUE = 135;
        public static final int LEDS_CONE_HUE = 28;
}
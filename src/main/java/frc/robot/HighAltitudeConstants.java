// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.chassis.DriveTrain.TransmissionMode;
import frc.robot.subsystems.chassis.DriveTrain.WheelState;

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

        public static final boolean DEBUG = true;

        /////////////////////////// DRIVE TRAIN ///////////////////////////////////////

        // The constant that helps the robot stay in the same angle while driving on an
        // assisted mode with the dragonfly. The higher the constant, the sharper the
        // correction.
        // 0.01 -> 0.05 -> 0.025
        public static double DRIVETRAIN_DRAGONFLY_TURN_CORRECTION = 0.01;

        // The sides of the robot normally move faster than the dragonfly module, this
        // constant indicates their rate. A constant of 0.60 means that the dragonfly
        // module has 60% of the speed of the sides.
        // 0.6 -> 0.7 -> 0.65
        public static double DRIVETRAIN_DRAGONFLY_SIDES_CORRECTION = 0.6;

        //// DEFAULT INITIAL PARAMETERS

        public static TransmissionMode DRIVETRAIN_INITIAL_TRANSMISSION_MODE = TransmissionMode.speed;
        public static WheelState DRIVETRAIN_INITIAL_DRAGONFLY_STATE = WheelState.Lowered;

        // Default braking mode, true for brake, false for coast.
        public static boolean DRIVETRAIN_MOTORS_BRAKING_MODE = true;

        ///// AUTOS

        /// Turning

        // When turning in autonomous at power 1, if the difference between the target
        // angle and the current angle (in degrees) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // turning speed.
        public static double DRIVETRAIN_AUTO_TURNING_BRAKING_DISTANCE = 100;
        // When turning in autonomous, if the difference between the target angle and
        // the current angle (in degrees) is less than this constant, it will be
        // considered on target.
        public static double DRIVETRAIN_AUTO_TURNING_ARRIVE_OFFSET = 3;

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

        ///// ENCODERS AND GEARBOX

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double DRIVETRAIN_PULSES_PER_REVOLUTION = 1;

        // In meters
        public static final double DRIVETRAIN_WHEEL_DIAMETER = 4 * 0.0254;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS.
        public static final double DRIVETRAIN_GEAR_RATIO_SPEED = 405.0 / 98.0;
        public static final double DRIVETRAIN_GEAR_RATIO_TORQUE = 225.0 / 28.0;

        // Use these constants to convert from encoder position to meters
        // (position*these constants = meters)
        public static final double DRIVETRAIN_METERS_PER_PULSE_TORQUE = Math.PI * DRIVETRAIN_WHEEL_DIAMETER
                        / (DRIVETRAIN_PULSES_PER_REVOLUTION * DRIVETRAIN_GEAR_RATIO_TORQUE);
        public static final double DRIVETRAIN_METERS_PER_PULSE_SPEED = Math.PI * DRIVETRAIN_WHEEL_DIAMETER
                        / (DRIVETRAIN_PULSES_PER_REVOLUTION * DRIVETRAIN_GEAR_RATIO_SPEED);

        ///////////////////////////// WRIST ////////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean WRIST_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double WRIST_PULSES_PER_REVOLUTION = 1;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / WRIST REVS
        public static final double WRIST_RATIO = 3249.0 / 49.0;

        public static final double WRIST_DEGREES_PER_PULSE = 360 / (WRIST_PULSES_PER_REVOLUTION * WRIST_RATIO);

        // When moving to a position at power 1, if the difference between the target
        // and the current position (in degrees) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // speed (from -1 to 1).
        public static final double WRIST_BRAKING_DEGREES = 15;

        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in meters) is less than this
        // constant, it will be considered on target.
        public static final double WRIST_ARRIVE_OFFSET = 4;

        ///////////////////////////// EXTENSOR /////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean EXTENSOR_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double EXTENSOR_PULSES_PER_REVOLUTION = 1.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / ARM REVS
        public static final double EXTENSOR_RATIO = 1.0 / 1.0;

        public static final double EXTENSOR_PITCH_DIAMETER_METERS = 1.0;

        public static final double EXTENSOR_METERS_PER_PULSE = (Math.PI * EXTENSOR_PITCH_DIAMETER_METERS)
                        / (EXTENSOR_PULSES_PER_REVOLUTION * EXTENSOR_RATIO);

        // When moving to a position at power 1, if the difference between the target
        // and the current position (in meters) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // speed (from -1 to 1).
        public static final double EXTENSOR_BRAKING_METERS = 15;
        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in meters) is less than this
        // constant, it will be considered on target.

        public static final double EXTENSOR_ARRIVE_OFFSET = 4;

        //////////////////////////////// ARM ///////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean ARM_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double ARM_PULSES_PER_REVOLUTION = 2048;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / ARM REVS
        public static final double ARM_RATIO = 94221.0 / 343.0;

        public static final double ARM_DEGREES_PER_PULSE = 360 / (ARM_PULSES_PER_REVOLUTION * ARM_RATIO);

        // When moving to a position at power 1, if the difference between the target
        // and the current position (in degrees) is less than this constant, it will
        // start braking. Note that this constant is proportional to the square of the
        // speed (from -1 to 1).
        public static final double ARM_BRAKING_DEGREES = 15;
        // When moving straight (straightMove()) in autonomous, if the difference
        // between the target and the current position (in degrees) is less than this
        // constant, it will be considered on target.

        public static final double ARM_ARRIVE_OFFSET = 4;

        //////////////////////// TRANSPORT CONSTANTS ///////////////////////////////////

        ////////// TOP ROW

        public static final double WRIST_TOP_ROW_DEGREES = -1.0;
        public static final double EXTENSOR_TOP_ROW_METERS = -1.0;
        public static final double ARM_TOP_ROW_DEGREES = -1.0;

        ////////// MIDDLE ROW

        public static final double WRIST_MIDDLE_ROW_DEGREES = -1.0;
        public static final double EXTENSOR_MIDDLE_ROW_METERS = -1.0;
        public static final double ARM_MIDDLE_ROW_DEGREES = -1.0;

        ////////// BOTTOM ROW

        public static final double WRIST_BOTTOM_ROW_DEGREES = -1.0;
        public static final double EXTENSOR_BOTTOM_ROW_METERS = -1.0;
        public static final double ARM_BOTTOM_ROW_DEGREES = -1.0;

        ///////////////////////////// GRIPPER //////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean GRIPPER_MOTORS_BRAKING_MODE = true;

        public static final double GRIPPER_CUBE_IN_SPEED = -0.2;
        public static final double GRIPPER_CUBE_OUT_SPEED = 1;
        public static final double GRIPPER_CONE_IN_SPEED = 0.2;
        public static final double GRIPPER_CONE_OUT_SPEED = -1;

        public static final double GRIPPER_DEFAULT_IN_SPEED = GRIPPER_CONE_IN_SPEED;
        public static final double GRIPPER_DEFAULT_OUT_SPEED = GRIPPER_CONE_OUT_SPEED;

        ///////////////////// LEDS ////////////////////////

        public static final int LEDS_9280_HUE = 64;
        public static final int LEDS_CUBE_HUE = 138;
        public static final int LEDS_CONE_HUE = 30;

}

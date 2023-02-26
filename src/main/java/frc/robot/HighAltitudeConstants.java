// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

        public static final boolean DEBUG = true;

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

        public static TransmissionMode DRIVETRAIN_INITIAL_TRANSMISSION_MODE = TransmissionMode.speed;
        public static WheelState DRIVETRAIN_INITIAL_DRAGONFLY_STATE = WheelState.Raised;

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

        /// Splines

        // When following a spline, this constant will help correct the error in y.
        public static final double SPLINE_DRIVE_ERROR_CORRECTION = 70;
        // When following a spline, if the difference between the target
        // and the current position (in meters) is less than this constant, it be
        // consider on target.
        public static final double DRIVETRAIN_SPLINE_ARRIVE_OFFSET = 0;
        // When following a spline, this constant will determine
        // how sharp the angle correction is. The higher the values, the sharper the
        // angle correction.
        public static final double DRIVETRAIN_SPLINE_ANGLE_CORRECTION = 0;
        public static final double SPLINE_SPEED_REDUCTION_BRAKING_DISTANCE = 0;

        /// Alignment
        public static final double DRIVETRAIN_ALIGN_MAX_SPEED = 0.5;

        /// Auto balancing

        // Units are in degrees and seconds

        // If the absolute value of the angular acceleration is smaller than this value,
        // The robot will be consider as stable on the charging station
        public static final double BALANCING_ACCELERATION_THRESHOLD = 180;

        // If the absolute value of the angle (pitch) is smaller than this value,
        // The robot will be consider as balanced on the charging station.
        // Be careful with small values (less than about 10°), because some bounciness
        // can occur.
        public static final double BALANCING_ANGLE_THRESHOLD = 20;

        // When trying to balance, this is the default power at which the robot will
        // move to
        // Try to balance the charging station. It is recommended to set it low to
        // improve accuracy.
        public static final double BALANCING_DEFAULT_POWER = 0.3;

        // To prevent the robot from falling off the charging station, it will not move
        // Unless it's properly aligned, that is to say, its angle is less than this
        // value.
        public static final double BALANCING_ALIGNED_THRESHOLD = 5;

        ///// ENCODERS AND GEARBOX

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double DRIVETRAIN_PULSES_PER_REVOLUTION = 1;

        // In meters
        public static final double DRIVETRAIN_WHEEL_DIAMETER = 4 * 0.0254;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS.
        public static final double DRIVETRAIN_GEAR_RATIO_SPEED = 100.0 / 21.0;
        public static final double DRIVETRAIN_GEAR_RATIO_TORQUE = 500.0 / 49.0;

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
        public static final double WRIST_RATIO = 1653.0 / 49.0;

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

        public static final double WRIST_UPPER_LIMIT_DEGREES = 999999999;
        public static final double WRIST_LOWER_LIMIT_DEGREES = -999999999;

        ///////////////////////////// EXTENSOR /////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean EXTENSOR_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double EXTENSOR_PULSES_PER_REVOLUTION = 1.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / ARM REVS
        public static final double EXTENSOR_RATIO = 42.0 / 1.0;

        public static final double EXTENSOR_PITCH_DIAMETER_METERS = 1.273 * 0.0254;

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

        public static final double EXTENSOR_UPPER_LIMIT_METERS = 999999999;
        public static final double EXTENSOR_LOWER_LIMIT_METERS = -999999999;

        //////////////////////////////// ARM ///////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean ARM_MOTORS_BRAKING_MODE = true;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double ARM_PULSES_PER_REVOLUTION = 2048;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. MOTOR REVS / ARM REVS
        public static final double ARM_RATIO = 31407.0 / 343.0;

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

        public static final double ARM_UPPER_LIMIT_DEGREES = 999999999;
        public static final double ARM_LOWER_LIMIT_DEGREES = -999999999;

        //////////////////////// TRANSPORT CONSTANTS ///////////////////////////////////

        ////////////////////// AUTONOMOUS MOVEMENT

        public static final double WRIST_AUTO_MAX_POWER = 0.2;
        public static final double EXTENSOR_AUTO_MAX_POWER = 0.2;
        public static final double ARM_AUTO_MAX_POWER = 0.2;

        /////////////////// CONE MODE ///////////////////

        ////////// TOP ROW

        public static final double WRIST_TOP_ROW_DEGREES_CONE = -1.0;
        public static final double EXTENSOR_TOP_ROW_METERS_CONE = -1.0;
        public static final double ARM_TOP_ROW_DEGREES_CONE = -1.0;

        ////////// MIDDLE ROW

        public static final double WRIST_MIDDLE_ROW_DEGREES_CONE = -1.0;
        public static final double EXTENSOR_MIDDLE_ROW_METERS_CONE = -1.0;
        public static final double ARM_MIDDLE_ROW_DEGREES_CONE = -1.0;

        ////////// BOTTOM ROW

        public static final double WRIST_BOTTOM_ROW_DEGREES_CONE = -1.0;
        public static final double EXTENSOR_BOTTOM_ROW_METERS_CONE = -1.0;
        public static final double ARM_BOTTOM_ROW_DEGREES_CONE = -1.0;

        ////////// GRAB FROM FEEDER

        public static final double WRIST_FEEDER_DEGREES_CONE = -1.0;
        public static final double EXTENSOR_FEEDER_METERS_CONE = -1.0;
        public static final double ARM_FEEDER_DEGREES_CONE = -1.0;

        ////////// GRAB FROM INTAKE

        public static final double WRIST_INTAKE_DEGREES_CONE = -1.0;
        public static final double EXTENSOR_INTAKE_METERS_CONE = -1.0;
        public static final double ARM_INTAKE_DEGREES_CONE = -1.0;

        /////////////////// CUBE MODE ///////////////////

        ////////// TOP ROW

        public static final double WRIST_TOP_ROW_DEGREES_CUBE = -1.0;
        public static final double EXTENSOR_TOP_ROW_METERS_CUBE = -1.0;
        public static final double ARM_TOP_ROW_DEGREES_CUBE = -1.0;

        ////////// MIDDLE ROW

        public static final double WRIST_MIDDLE_ROW_DEGREES_CUBE = -1.0;
        public static final double EXTENSOR_MIDDLE_ROW_METERS_CUBE = -1.0;
        public static final double ARM_MIDDLE_ROW_DEGREES_CUBE = -1.0;

        ////////// BOTTOM ROW

        public static final double WRIST_BOTTOM_ROW_DEGREES_CUBE = -1.0;
        public static final double EXTENSOR_BOTTOM_ROW_METERS_CUBE = -1.0;
        public static final double ARM_BOTTOM_ROW_DEGREES_CUBE = -1.0;

        ////////// GRAB FROM FEEDER

        public static final double WRIST_FEEDER_DEGREES_CUBE = -1.0;
        public static final double EXTENSOR_FEEDER_METERS_CUBE = -1.0;
        public static final double ARM_FEEDER_DEGREES_CUBE = -1.0;

        ////////// GRAB FROM INTAKE

        public static final double WRIST_INTAKE_DEGREES_CUBE = -1.0;
        public static final double EXTENSOR_INTAKE_METERS_CUBE = -1.0;
        public static final double ARM_INTAKE_DEGREES_CUBE = -1.0;

        ///////////////////////////// GRIPPER //////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean GRIPPER_MOTORS_BRAKING_MODE = true;

        public static final double GRIPPER_CUBE_IN_SPEED = -0.2;
        public static final double GRIPPER_CUBE_OUT_SPEED = 1;
        public static final double GRIPPER_CONE_IN_SPEED = 0.2;
        public static final double GRIPPER_CONE_OUT_SPEED = -1;

        public static final double GRIPPER_DEFAULT_IN_SPEED = GRIPPER_CONE_IN_SPEED;
        public static final double GRIPPER_DEFAULT_OUT_SPEED = GRIPPER_CONE_OUT_SPEED;

        ///////////////////////////// INTAKE //////////////////////////////////////////

        // Default braking mode, true for brake, false for coast.
        public static final boolean INTAKE_MOTORS_BRAKING_MODE = true;

        public static final IntakePosition INTAKE_INITIAL_POSITION = IntakePosition.STORED;

        public static final double INTAKE_IN_SPEED = 0.5;
        public static final double INTAKE_OUT_SPEED = -0.5;

        ///////////////////// LEDS ////////////////////////

        public static final int LEDS_9280_HUE = 64;
        public static final int LEDS_CUBE_HUE = 138;
        public static final int LEDS_CONE_HUE = 28;
}

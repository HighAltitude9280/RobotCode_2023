// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain.TransmissionMode;
import frc.robot.subsystems.DriveTrain.WheelState;

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

    //////////////////////// DRIVETRAIN///////////////////////////////////

    // The constant that helps the robot stay in the same angle while driving on an
    // assisted mode with the dragonfly. It helps correct the current error in the angle.
    // The higher this constant, the more it will turn to correct, specially after braking.  
    public static final double DRIVETRAIN_DRAGONFLY_ANGLE_CORRECTION = 0.01;
    // This constants also helps the robot stay in the same angle while driving on an 
    // assited mode with the dragonfly. It helps predict the error that will be generated
    // by the dragonfly and corrects based on that. The higher the constant, the more it will
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


}

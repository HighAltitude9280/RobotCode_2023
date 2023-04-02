package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;

public class RobotMap {

        public static final int REVPH_MODULE_ID = 62;

        ////////////////////////// SWERVE //////////////////////////

        ///// FRONT LEFT
        // DRIVE
        public static final int SWERVE_FRONT_LEFT_DRIVE_MOTOR_PORT = 1;
        public static final TypeOfMotor SWERVE_FRONT_LEFT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
        public static final boolean SWERVE_FRONT_LEFT_DRIVE_ENCODER_INVERTED = false;
        // DIRECTION
        public static final int SWERVE_FRONT_LEFT_DIRECTION_MOTOR_PORT = 2;
        public static final TypeOfMotor SWERVE_FRONT_LEFT_DIRECTION_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_FRONT_LEFT_DIRECTION_MOTOR_INVERTED = true; // false // false
        public static final boolean SWERVE_FRONT_LEFT_DIRECTION_ENCODER_INVERTED = false; // false // true
        // TALON ENCODER
        public static final int SWERVE_FRONT_LEFT_ENCODED_TALON_PORT = 3;
        public static final double SWERVE_FRONT_LEFT_DIRECTION_ENCODER_OFFSET_PULSES = 1657; // 1679
        public static final boolean SWERVE_FRONT_LEFT_ENCODED_TALON_INVERTED = false; // true

        ///// FRONT RIGHT
        // DRIVE
        public static final int SWERVE_FRONT_RIGHT_DRIVE_MOTOR_PORT = 4;
        public static final TypeOfMotor SWERVE_FRONT_RIGHT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
        public static final boolean SWERVE_FRONT_RIGHT_DRIVE_ENCODER_INVERTED = false; // true
        // DIRECTION
        public static final int SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_PORT = 5;
        public static final TypeOfMotor SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_INVERTED = true; // false // false
        public static final boolean SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_INVERTED = false; // false // true
        // TALON ENCODER
        public static final int SWERVE_FRONT_RIGHT_ENCODED_TALON_PORT = 6;
        public static final double SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES = 124;
        public static final boolean SWERVE_FRONT_RIGHT_ENCODED_TALON_INVERTED = false; // true

        ///// BACK LEFT
        // DRIVE
        public static final int SWERVE_BACK_LEFT_DRIVE_MOTOR_PORT = 7;
        public static final TypeOfMotor SWERVE_BACK_LEFT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
        public static final boolean SWERVE_BACK_LEFT_DRIVE_ENCODER_INVERTED = false;
        // DIRECTION
        public static final int SWERVE_BACK_LEFT_DIRECTION_MOTOR_PORT = 8;
        public static final TypeOfMotor SWERVE_BACK_LEFT_DIRECTION_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_BACK_LEFT_DIRECTION_MOTOR_INVERTED = true; // false //false
        public static final boolean SWERVE_BACK_LEFT_DIRECTION_ENCODER_INVERTED = false; // false // true
        // TALON ENCODER
        public static final int SWERVE_BACK_LEFT_ENCODED_TALON_PORT = 9;
        public static final double SWERVE_BACK_LEFT_DIRECTION_ENCODER_OFFSET_PULSES = 3933;
        public static final boolean SWERVE_BACK_LEFT_ENCODED_TALON_INVERTED = false; // true

        ///// BACK RIGHT
        // DRIVE
        public static final int SWERVE_BACK_RIGHT_DRIVE_MOTOR_PORT = 10;
        public static final TypeOfMotor SWERVE_BACK_RIGHT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_BACK_RIGHT_DRIVE_MOTOR_INVERTED = true;
        public static final boolean SWERVE_BACK_RIGHT_DRIVE_ENCODER_INVERTED = false; // true
        // DIRECTION
        public static final int SWERVE_BACK_RIGHT_DIRECTION_MOTOR_PORT = 11;
        public static final TypeOfMotor SWERVE_BACK_RIGHT_DIRECTION_MOTOR_TYPE = TypeOfMotor.TALON_FX;
        public static final boolean SWERVE_BACK_RIGHT_DIRECTION_MOTOR_INVERTED = true; // false // false
        public static final boolean SWERVE_BACK_RIGHT_DIRECTION_ENCODER_INVERTED = false; // false //true
        // TALON ENCODER
        public static final int SWERVE_BACK_RIGHT_ENCODED_TALON_PORT = 12;
        public static final double SWERVE_BACK_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES = 1572; // 1566
        public static final boolean SWERVE_BACK_RIGHT_ENCODED_TALON_INVERTED = false; // true

        //////////////////////// DRIVE TRAIN////////////////////////

        // Transmission
        public static final boolean DRIVETRAIN_TRANSMISSION_IS_AVAILABLE = true;
        public static final int[] DRIVETRAIN_TRANSMISSION_SOLENOID_PORTS = { 11, 4 };
        public static final PneumaticsModuleType DRIVETRAIN_TRANSMISSION_MODULE_TYPE = PneumaticsModuleType.REVPH;

        public static final Value DRIVETRAIN_TRANSMISSION_SPEED = Value.kReverse;
        public static final Value DRIVETRAIN_TRANSMISSION_TORQUE = Value.kForward;

        // Left side
        public static final int[] DRIVETRAIN_LEFT_MOTOR_PORTS = { 1, 2, 3 }; // 1 2 3
        public static final int[] DRIVETRAIN_LEFT_INVERTED_MOTORS_PORTS = { 1, 2, 3 }; // 1 2 3
        public static final boolean DRIVETRAIN_LEFT_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] DRIVETRAIN_LEFT_MOTOR_TYPES = { TypeOfMotor.TALON_FX, TypeOfMotor.TALON_FX,
                        TypeOfMotor.TALON_FX };

        // Right side
        public static final int[] DRIVETRAIN_RIGHT_MOTOR_PORTS = { 4, 5, 6 }; // 4 5
        public static final int[] DRIVETRAIN_RIGHT_INVERTED_MOTORS_PORTS = {}; // -
        public static final boolean DRIVETRAIN_RIGHT_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] DRIVETRAIN_RIGHT_MOTOR_TYPES = { TypeOfMotor.TALON_FX, TypeOfMotor.TALON_FX,
                        TypeOfMotor.TALON_FX };

        // Dragonfly
        public static final boolean DRIVETRAIN_DRAGONFLY_SOLENOID_IS_AVAILABLE = false;
        public static final int[] DRIVETRAIN_DRAGONFLY_SOLENOID_PORTS = { 6, 9 };
        public static final PneumaticsModuleType DRIVETRAIN_DRAGONFLY_SOLENOID_MODULE_TYPE = PneumaticsModuleType.REVPH;

        public static final Value DRIVETRAIN_DRAGONFLY_LOWERED = Value.kForward;
        public static final Value DRIVETRAIN_DRAGONFLY_RAISED = Value.kReverse;

        public static final boolean DRIVETRAIN_DRAGONFLY_WHEEL_IS_AVAILABLE = false;
        public static final int[] DRIVETRAIN_DRAGONFLY_MOTOR_PORTS = { 7 };
        public static final int[] DRIVETRAIN_DRAGONFLY_INVERTED_MOTORS_PORTS = {};
        public static final boolean DRIVETRAIN_DRAGONFLY_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] DRIVETRAIN_DRAGONFLY_MOTOR_TYPES = { TypeOfMotor.TALON_FX };

        //////////////////////// WRIST ////////////////////////

        public static final int[] WRIST_MOTOR_PORTS = { 22 };
        public static final int[] WRIST_INVERTED_MOTORS_PORTS = {};
        public static final boolean WRIST_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] WRIST_MOTOR_TYPES = { TypeOfMotor.TALON_FX };

        //////////////////////// EXTENSOR ////////////////////////

        public static final int[] EXTENSOR_MOTOR_PORTS = { 20 };
        public static final int[] EXTENSOR_INVERTED_MOTORS_PORTS = {};
        public static final boolean EXTENSOR_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] EXTENSOR_MOTOR_TYPES = { TypeOfMotor.TALON_FX };

        //////////////////////// ARM ////////////////////////

        public static final int[] ARM_MOTOR_PORTS = { 21 };
        public static final int[] ARM_INVERTED_MOTORS_PORTS = {};
        public static final boolean ARM_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] ARM_MOTOR_TYPES = { TypeOfMotor.TALON_FX };

        //////////////////////// GRIPPER ////////////////////////

        public static final int[] GRIPPER_MOTOR_PORTS = { 30 };
        public static final int[] GRIPPER_INVERTED_MOTORS_PORTS = { 30 };
        public static final boolean GRIPPER_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] GRIPPER_MOTOR_TYPES = { TypeOfMotor.TALON_FX };

        // Limit switch
        public static final boolean GRIPPER_LIMIT_SWITCH_IS_AVAILABLE = false;
        public static final int GRIPPER_LIMIT_SWITCH_PORT = 0;

        //////////////////////// INTAKE ////////////////////////

        public static final int[] INTAKE_MOTOR_PORTS = { 10, 11 };
        public static final int[] INTAKE_INVERTED_MOTORS_PORTS = { 10 };
        public static final boolean INTAKE_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] INTAKE_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS,
                        TypeOfMotor.CAN_SPARK_BRUSHLESS };

        public static final int[] INTAKE_SOLENOID_PORTS = { 10, 5 };
        public static final PneumaticsModuleType INTAKE_SOLENOID_MODULE_TYPE = PneumaticsModuleType.REVPH;

        public static final Value INTAKE_LOWERED_VALUE = Value.kForward;
        public static final Value INTAKE_STORED_VALUE = Value.kReverse;
}
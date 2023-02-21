package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;

public class RobotMap {

        //////////////////////// DRIVE TRAIN ////////////////////////

        // Transmission
        public static final boolean DRIVETRAIN_TRANSMISSION_IS_AVAILABLE = false;
        public static final int[] DRIVETRAIN_TRANSMISSION_SOLENOID_PORTS = { 0, 1 };
        public static final PneumaticsModuleType DRIVETRAIN_TRANSMISSION_MODULE_TYPE = PneumaticsModuleType.REVPH;

        public static final Value DRIVETRAIN_TRANSMISSION_SPEED = Value.kReverse;
        public static final Value DRIVETRAIN_TRANSMISSION_TORQUE = Value.kForward;

        // Left side
        public static final int[] DRIVETRAIN_LEFT_MOTOR_PORTS = { 1, 2 };
        public static final int[] DRIVETRAIN_LEFT_INVERTED_MOTORS_PORTS = {};
        public static final boolean DRIVETRAIN_LEFT_ENCODER_IS_INVERTED = true;
        public static final TypeOfMotor[] DRIVETRAIN_LEFT_MOTOR_TYPES = { TypeOfMotor.TALON_SRX,
                        TypeOfMotor.TALON_SRX };

        // Right side
        public static final int[] DRIVETRAIN_RIGHT_MOTOR_PORTS = { 3, 4 };
        public static final int[] DRIVETRAIN_RIGHT_INVERTED_MOTORS_PORTS = {};
        public static final boolean DRIVETRAIN_RIGHT_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] DRIVETRAIN_RIGHT_MOTOR_TYPES = { TypeOfMotor.TALON_SRX,
                        TypeOfMotor.TALON_SRX };

        // Dragonfly
        public static final boolean DRIVETRAIN_DRAGONFLY_SOLENOID_AVAILABLE = false;
        public static final boolean DRIVETRAIN_DRAGONFLY_WHEEL_AVAILABLE = false;
        public static final int[] DRIVETRAIN_DRAGONFLY_SOLENOID_PORTS = { 2, 3 };
        public static final PneumaticsModuleType DRIVETRAIN_DRAGONFLY_SOLENOID_MODULE_TYPE = PneumaticsModuleType.REVPH;

        public static final Value DRIVETRAIN_DRAGONFLY_LOWERED = Value.kForward;
        public static final Value DRIVETRAIN_DRAGONFLY_RAISED = Value.kReverse;

        public static final int[] DRIVETRAIN_DRAGONFLY_MOTOR_PORTS = { 5 };
        public static final int[] DRIVETRAIN_DRAGONFLY_INVERTED_MOTORS_PORTS = {};
        public static final boolean DRIVETRAIN_DRAGONFLY_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] DRIVETRAIN_DRAGONFLY_MOTOR_TYPES = { TypeOfMotor.TALON_SRX };

        //////////////////////// WRIST ////////////////////////

        public static final int[] WRIST_MOTOR_PORTS = { 12 };
        public static final int[] WRIST_INVERTED_MOTORS_PORTS = {};
        public static final boolean WRIST_ENCODER_IS_INVERTED = true;
        public static final TypeOfMotor[] WRIST_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };

        //////////////////////// EXTENSOR ////////////////////////

        public static final int[] EXTENSOR_MOTOR_PORTS = { 15 };
        public static final int[] EXTENSOR_INVERTED_MOTORS_PORTS = { 15 };
        public static final boolean EXTENSOR_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] EXTENSOR_MOTOR_TYPES = { TypeOfMotor.TALON_SRX };

        //////////////////////// ARM ////////////////////////

        public static final int[] ARM_MOTOR_PORTS = { 11 };
        public static final int[] ARM_INVERTED_MOTORS_PORTS = {};
        public static final boolean ARM_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] ARM_MOTOR_TYPES = { TypeOfMotor.TALON_FX };

        //////////////////////// GRIPPER ////////////////////////

        public static final int[] GRIPPER_MOTOR_PORTS = { 30 };
        public static final int[] GRIPPER_INVERTED_MOTORS_PORTS = {};
        public static final boolean GRIPPER_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] GRIPPER_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };

        // Limit switch
        public static final boolean GRIPPER_LIMIT_SWITCH_IS_AVAILABLE = false;
        public static final int GRIPPER_LIMIT_SWITCH_PORT = 0;

        //////////////////////// INTAKE ////////////////////////

        public static final int[] INTAKE_MOTOR_PORTS = { 6, 7 };
        public static final int[] INTAKE_INVERTED_MOTORS_PORTS = { 7 };
        public static final boolean INTAKE_ENCODER_IS_INVERTED = false;
        public static final TypeOfMotor[] INTAKE_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS,
                        TypeOfMotor.CAN_SPARK_BRUSHLESS };

}

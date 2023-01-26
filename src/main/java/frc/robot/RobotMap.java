package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;


public class RobotMap {

    ////////////////////////DRIVE TRAIN////////////////////////

    //Transmission
    public static final boolean DRIVETRAIN_TRANSMISSION_IS_AVAILABLE = false;
    public static final int[] DRIVETRAIN_TRANSMISSION_SOLENOID_PORTS = {0,1};
    public static final PneumaticsModuleType DRIVETRAIN_TRANSMISSION_MODULE_TYPE 
        = PneumaticsModuleType.REVPH;

    public static final Value DRIVETRAIN_TRANSMISSION_SPEED = Value.kReverse;
    public static final Value DRIVETRAIN_TRANSMISSION_TORQUE = Value.kForward;

    //Left side
    public static final int[] DRIVETRAIN_LEFT_MOTOR_PORTS = {12,14};
    public static final int[] DRIVETRAIN_LEFT_INVERTED_MOTORS_PORTS = {12,14};
    public static final boolean DRIVETRAIN_LEFT_ENCODER_IS_INVERTED = true;
    public static final TypeOfMotor[] DRIVETRAIN_LEFT_MOTOR_TYPES 
        = {TypeOfMotor.VICTOR_SPX, TypeOfMotor.VICTOR_SPX};
    
    //Right side
    public static final int[] DRIVETRAIN_RIGHT_MOTOR_PORTS = {13,4};
    public static final int[] DRIVETRAIN_RIGHT_INVERTED_MOTORS_PORTS = {};
    public static final boolean DRIVETRAIN_RIGHT_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] DRIVETRAIN_RIGHT_MOTOR_TYPES 
        = {TypeOfMotor.VICTOR_SPX, TypeOfMotor.VICTOR_SPX};
    
    //Dragonfly 
    public static final boolean DRIVETRAIN_DRAGONFLY_IS_AVAILABLE = false;
    public static final int[] DRIVETRAIN_DRAGONFLY_SOLENOID_PORTS = {2,3};
    public static final PneumaticsModuleType DRIVETRAIN_DRAGONFLY_SOLENOID_MODULE_TYPE
        = PneumaticsModuleType.REVPH;
        
    public static final Value DRIVETRAIN_DRAGONFLY_LOWERED = Value.kForward;
    public static final Value DRIVETRAIN_DRAGONFLY_RAISED = Value.kReverse;

    public static final int[] DRIVETRAIN_DRAGONFLY_MOTOR_PORTS = {};
    public static final int[] DRIVETRAIN_DRAGONFLY_INVERTED_MOTORS_PORTS = {};
    public static final boolean DRIVETRAIN_DRAGONFLY_ENCODER_IS_INVERTED = false;
    public static final TypeOfMotor[] DRIVETRAIN_DRAGONFLY_MOTOR_TYPES = {};
}

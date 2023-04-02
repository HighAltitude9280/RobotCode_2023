// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Gripper extends SubsystemBase {
  HighAltitudeMotorGroup gripperMotors;
  DigitalInput cubeLimitSwitch;

  /** Creates a new Gripper. */
  public Gripper() {

    gripperMotors = new HighAltitudeMotorGroup(RobotMap.GRIPPER_MOTOR_PORTS, RobotMap.GRIPPER_INVERTED_MOTORS_PORTS,
        RobotMap.GRIPPER_MOTOR_TYPES);
    gripperMotors.setEncoderInverted(RobotMap.GRIPPER_ENCODER_IS_INVERTED);
    gripperMotors.setBrakeMode(HighAltitudeConstants.GRIPPER_MOTORS_BRAKING_MODE);

    if (RobotMap.GRIPPER_LIMIT_SWITCH_IS_AVAILABLE)
      cubeLimitSwitch = new DigitalInput(RobotMap.GRIPPER_LIMIT_SWITCH_PORT);
  }

  public void driveGripper(double speed) {
    gripperMotors.setAll(speed);
    // Robot.debugPrint("GripperSpeed: " + speed);
  }

  public void stopGripper() {
    gripperMotors.setAll(0);
  }

  public boolean getCubeLimitSwitch() {
    if (RobotMap.GRIPPER_LIMIT_SWITCH_IS_AVAILABLE)
      return cubeLimitSwitch.get();
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

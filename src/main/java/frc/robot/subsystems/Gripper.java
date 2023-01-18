// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Gripper extends SubsystemBase {
  HighAltitudeMotorGroup gripperMotors;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperMotors = new HighAltitudeMotorGroup(RobotMap.GRIPPER_MOTOR_PORTS, RobotMap.GRIPPER_INVERTED_MOTORS_PORTS,
        RobotMap.GRIPPER_MOTOR_TYPES);
    gripperMotors.setEncoderInverted(RobotMap.GRIPPER_ENCODER_IS_INVERTED);
    gripperMotors.setBrakeMode(HighAltitudeConstants.GRIPPER_MOTORS_BRAKING_MODE);
  }

  public void driveGripper(double speed) {
    gripperMotors.setAll(speed);
    Robot.debug("GripperSpeed: " + speed);
  }

  public void stopGripper() {
    gripperMotors.setAll(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

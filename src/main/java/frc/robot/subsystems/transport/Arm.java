// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Arm extends SubsystemBase {

  HighAltitudeMotorGroup armMotors;
  double armEncoderPosition, armPositionDegrees;

  /** Creates a new Arm. */
  public Arm() {
    armMotors = new HighAltitudeMotorGroup(RobotMap.ARM_MOTOR_PORTS, RobotMap.ARM_INVERTED_MOTORS_PORTS,
        RobotMap.ARM_MOTOR_TYPES);
    armMotors.setEncoderInverted(RobotMap.ARM_ENCODER_IS_INVERTED);
    armMotors.setBrakeMode(HighAltitudeConstants.ARM_MOTORS_BRAKING_MODE);
  }

  public void driveArm(double speed) {
    armMotors.setAll(speed);
    Robot.debug("Armpower: " + speed);
  }

  public boolean moveTo(double targetDegrees, double maxPower) {
    double delta = targetDegrees - armPositionDegrees;

    if (Math.abs(delta) < HighAltitudeConstants.ARM_ARRIVE_OFFSET) {
      armMotors.setAll(0);
      return true;
    }

    double power = delta / (HighAltitudeConstants.ARM_BRAKING_DEGREES * maxPower * maxPower);
    power = Math.clamp(power, -1, 1) * maxPower;

    armMotors.setAll(power);
    return false;
  }

  @Override
  public void periodic() {
    armEncoderPosition = armMotors.getEncoderPosition();
    armPositionDegrees = armEncoderPosition * HighAltitudeConstants.ARM_DEGREES_PER_PULSE;
    // Robot.debug("ArmPos:" + armEncoderPosition + " ArmDeg: " +
    // armPositionDegrees);
  }
}

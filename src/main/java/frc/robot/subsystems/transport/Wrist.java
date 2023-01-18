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

public class Wrist extends SubsystemBase {

  HighAltitudeMotorGroup wristMotors;
  double wristEncoderPosition, wristPositionDegrees;

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotors = new HighAltitudeMotorGroup(RobotMap.WRIST_MOTOR_PORTS, RobotMap.WRIST_INVERTED_MOTORS_PORTS,
        RobotMap.WRIST_MOTOR_TYPES);
    wristMotors.setEncoderInverted(RobotMap.WRIST_ENCODER_IS_INVERTED);
    wristMotors.setBrakeMode(HighAltitudeConstants.WRIST_MOTORS_BRAKING_MODE);
  }

  public void driveWrist(double speed) {
    wristMotors.setAll(speed);
    Robot.debug("WristPower: " + speed);
  }

  public boolean moveTo(double targetDegrees, double maxPower) {
    double delta = targetDegrees - wristPositionDegrees;

    if (Math.abs(delta) < HighAltitudeConstants.WRIST_ARRIVE_OFFSET) {
      wristMotors.setAll(0);
      return true;
    }

    double power = delta / (HighAltitudeConstants.WRIST_BRAKING_DEGREES * maxPower * maxPower);
    power = Math.clamp(power, -1, 1) * maxPower;

    wristMotors.setAll(power);
    return false;
  }

  @Override
  public void periodic() {
    wristEncoderPosition = wristMotors.getEncoderPosition();
    wristPositionDegrees = wristEncoderPosition * HighAltitudeConstants.WRIST_DEGREES_PER_PULSE;
    // Robot.debug("WristPos:" + wristEncoderPosition + " WristDeg: " +
    // wristPositionDegrees);
  }
}

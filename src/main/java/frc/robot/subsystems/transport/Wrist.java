// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // resetEncoders();
  }

  public void driveWrist(double speed) {
    boolean isBelowLimitsAndSpeedIsNegative = (wristPositionDegrees < HighAltitudeConstants.WRIST_LOWER_LIMIT_DEGREES
        && speed < 0);
    boolean isOverLimitsAndSpeedIsPositive = (wristPositionDegrees > HighAltitudeConstants.WRIST_UPPER_LIMIT_DEGREES
        && speed > 0);

    if (Robot.getRobotContainer().getShouldManualHaveLimits()
        && (isBelowLimitsAndSpeedIsNegative || isOverLimitsAndSpeedIsPositive)) {
      wristMotors.setAll(0);
      Robot.debugPrint("YA TE PASASTE DEL LIMITE DE LA MUÑECA YA MAMÓ");
      return;
    }

    wristMotors.setAll(speed);
    // Robot.debugPrint("WristPower: " + speed);
  }

  public boolean moveTo(double targetDegrees, double maxPower) {
    double delta = targetDegrees - wristPositionDegrees;

    if (Math.abs(delta) < HighAltitudeConstants.WRIST_ARRIVE_OFFSET) {
      wristMotors.setAll(0);
      return true;
    }

    double power = delta / (HighAltitudeConstants.WRIST_BRAKING_DEGREES *
        maxPower * maxPower);
    power = Math.clamp(power, -1, 1) * maxPower;

    wristMotors.setAll(power);
    return false;
  }

  public void resetEncoders() {
    wristMotors.resetEncoder();
  }

  @Override
  public void periodic() {
    wristEncoderPosition = wristMotors.getEncoderPosition();
    wristPositionDegrees = wristEncoderPosition * HighAltitudeConstants.WRIST_DEGREES_PER_PULSE + 45.64;

    Robot.debugNumberSmartDashboard("Wrist Encoder", wristEncoderPosition);
    SmartDashboard.putNumber("Wrist Degrees", wristPositionDegrees);
    // Robot.debug("WristPos:" + wristEncoderPosition + " WristDeg: " +
    // wristPositionDegrees);
  }

  public double getCurrentAngle() {
    return wristPositionDegrees;
  }
}

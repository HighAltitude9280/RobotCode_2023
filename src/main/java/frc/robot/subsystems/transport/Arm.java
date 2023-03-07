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
  double currentArmEncoderPosition, armPositionDegrees;

  /** Creates a new Arm. */
  public Arm() {
    armMotors = new HighAltitudeMotorGroup(RobotMap.ARM_MOTOR_PORTS, RobotMap.ARM_INVERTED_MOTORS_PORTS,
        RobotMap.ARM_MOTOR_TYPES);
    armMotors.setEncoderInverted(RobotMap.ARM_ENCODER_IS_INVERTED);
    armMotors.setBrakeMode(HighAltitudeConstants.ARM_MOTORS_BRAKING_MODE);

    // resetEncoders();
  }

  public void driveArm(double speed) {
    boolean isBelowLimitsAndSpeedIsNegative = (armPositionDegrees < HighAltitudeConstants.ARM_LOWER_LIMIT_DEGREES
        && speed < 0);
    boolean isOverLimitsAndSpeedIsPositive = (armPositionDegrees > HighAltitudeConstants.ARM_UPPER_LIMIT_DEGREES
        && speed > 0);

    if (Robot.getRobotContainer().getShouldManualHaveLimits()
        && (isBelowLimitsAndSpeedIsNegative || isOverLimitsAndSpeedIsPositive)) {
      armMotors.setAll(0);
      Robot.debugPrint("YA TE PASASTE DEL LIMITE DEL BRAZO YA MAMÓ");
      return;
    }
    armMotors.setAll(speed);
    // Robot.debugPrint("Armpower: " + speed);
  }

  public boolean moveTo(double targetDegrees, double maxPower) {
    double delta = targetDegrees - armPositionDegrees;

    if (Math.abs(delta) < HighAltitudeConstants.ARM_ARRIVE_OFFSET) {
      armMotors.setAll(0);
      return true;
    }

    // double power = delta / (HighAltitudeConstants.ARM_BRAKING_DEGREES * maxPower
    // * maxPower);
    double power = delta / HighAltitudeConstants.ARM_BRAKING_DEGREES;
    power = Math.clamp(power, -1, 1) * maxPower;

    armMotors.setAll(power);
    Robot.debugPrint("ARM MOVING TOOOOO: " + power);
    return false;
  }

  public void resetEncoders() {
    armMotors.resetEncoder();
  }

  @Override
  public void periodic() {
    currentArmEncoderPosition = armMotors.getEncoderPosition();
    armPositionDegrees = currentArmEncoderPosition * HighAltitudeConstants.ARM_DEGREES_PER_PULSE;

    // Robot.debugNumberSmartDashboard("Arm Encoder", currentArmEncoderPosition);
    Robot.debugNumberSmartDashboard("Arm Degrees", armPositionDegrees);
    // Robot.debug("ArmPos:" + armEncoderPosition + " ArmDeg: " +
    // armPositionDegrees);
  }

  public double getCurrentAngle() {
    return armPositionDegrees;
  }
}

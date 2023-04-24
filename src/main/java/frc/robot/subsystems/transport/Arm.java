// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Arm extends SubsystemBase {

  HighAltitudeMotorGroup armMotors;
  double currentArmEncoderPosition, armPositionDegrees;

  // Arm odometry
  Translation2d armEndPointPos;

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

    double power = delta / (HighAltitudeConstants.ARM_BRAKING_DEGREES * maxPower
        * maxPower);
    // double power = delta / HighAltitudeConstants.ARM_BRAKING_DEGREES;
    power = Math.clamp(power, -1, 1) * maxPower;

    armMotors.setAll(power);
    return false;
  }

  public void resetEncoders() {
    armMotors.resetEncoder();
  }

  @Override
  public void periodic() {
    currentArmEncoderPosition = armMotors.getEncoderPosition();
    armPositionDegrees = currentArmEncoderPosition * HighAltitudeConstants.ARM_DEGREES_PER_PULSE;

    updateArmOdometry();
    Robot.debugNumberSmartDashboard("Arm Encoder", currentArmEncoderPosition);
    SmartDashboard.putNumber("Arm Degrees", armPositionDegrees);
    Robot.debugNumberSmartDashboard("Arm Y", armEndPointPos.getY());
    Robot.debugNumberSmartDashboard("Arm x", armEndPointPos.getX());
    // Robot.debug("ArmPos:" + armEncoderPosition + " ArmDeg: " +
    // armPositionDegrees);
  }

  public double getCurrentAngle() {
    return armPositionDegrees;
  }

  private void updateArmOdometry() {
    Translation2d carriageTraslation2d = new Translation2d(
        Robot.getRobotContainer().getExtensor().getCurrentDistance(), 0)
        .rotateBy(Rotation2d.fromDegrees(145));
    carriageTraslation2d = carriageTraslation2d.plus(HighAltitudeConstants.ARM_CARRIAGE_ZERO_TRANSLATION2D_METERS);

    Translation2d armPivotPoint = carriageTraslation2d
        .plus(HighAltitudeConstants.ARM_CARRIAGE_BOTTOM_TO_PIVOT_TRANSLATION2D_METERS);

    Rotation2d armAngleFromGround = Rotation2d.fromDegrees(getCurrentAngle() + HighAltitudeConstants.ARM_INTIAL_ANGLE);
    Translation2d armRodRotated = HighAltitudeConstants.ARM_ROD_TRANSLATION2D_METERS.rotateBy(armAngleFromGround);

    armEndPointPos = armPivotPoint.plus(armRodRotated);
  }
}

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

public class Extensor extends SubsystemBase {

  HighAltitudeMotorGroup extensorMotors;
  double extensorEncoderPosition, extensorPositionMeters;

  /** Creates a new Extensor. */
  public Extensor() {
    extensorMotors = new HighAltitudeMotorGroup(RobotMap.EXTENSOR_MOTOR_PORTS, RobotMap.EXTENSOR_INVERTED_MOTORS_PORTS,
        RobotMap.EXTENSOR_MOTOR_TYPES);
    extensorMotors.setEncoderInverted(RobotMap.EXTENSOR_ENCODER_IS_INVERTED);
    extensorMotors.setBrakeMode(HighAltitudeConstants.EXTENSOR_MOTORS_BRAKING_MODE);
  }

  public void driveExtensor(double speed) {
    extensorMotors.setAll(speed);
    Robot.debug("ExtensorPower: " + speed);
  }

  public boolean moveTo(double targetMeters, double maxPower) {
    double delta = targetMeters - extensorPositionMeters;

    if (Math.abs(delta) < HighAltitudeConstants.EXTENSOR_ARRIVE_OFFSET) {
      extensorMotors.setAll(0);
      return true;
    }

    double power = delta / (HighAltitudeConstants.EXTENSOR_BRAKING_METERS * maxPower * maxPower);
    power = Math.clamp(power, -1, 1) * maxPower;

    extensorMotors.setAll(power);
    return false;
  }

  @Override
  public void periodic() {
    extensorEncoderPosition = extensorMotors.getEncoderPosition();
    extensorPositionMeters = extensorEncoderPosition * HighAltitudeConstants.EXTENSOR_METERS_PER_PULSE;

    SmartDashboard.putNumber("Extensor Encoder", extensorEncoderPosition);
    SmartDashboard.putNumber("Extensor Meters", extensorPositionMeters);
    // Robot.debug("ExtensorPos:" + extensorEncoderPosition + " ExtensorDeg: " +
    // extensorPositionDegrees);
  }
}

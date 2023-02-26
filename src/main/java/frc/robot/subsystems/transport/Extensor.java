// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorController;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Extensor extends SubsystemBase {

  HighAltitudeMotorGroup extensorMotors;
  HighAltitudeMotorController xd;
  double extensorEncoderPosition, extensorPositionMeters;

  /** Creates a new Extensor. */
  public Extensor() {

    // xd = new HighAltitudeMotorController(RobotMap.EXTENSOR_MOTOR_PORTS[0],
    // RobotMap.EXTENSOR_MOTOR_TYPES[0]);

    // extensorMotors = new HighAltitudeMotorGroup(xd);
    extensorMotors = new HighAltitudeMotorGroup(RobotMap.EXTENSOR_MOTOR_PORTS,
        RobotMap.EXTENSOR_INVERTED_MOTORS_PORTS,
        RobotMap.EXTENSOR_MOTOR_TYPES);
    extensorMotors.setEncoderInverted(RobotMap.EXTENSOR_ENCODER_IS_INVERTED);
    extensorMotors.setBrakeMode(HighAltitudeConstants.EXTENSOR_MOTORS_BRAKING_MODE);
    // resetEncoders();
  }

  public void driveExtensor(double speed) {
    if (extensorPositionMeters < HighAltitudeConstants.EXTENSOR_LOWER_LIMIT_METERS
        || extensorPositionMeters > HighAltitudeConstants.EXTENSOR_UPPER_LIMIT_METERS)
      return;
    Robot.debugPrint("ExtensorPower: " + speed);
    extensorMotors.setAll(speed);
  }

  public boolean moveTo(double targetMeters, double maxPower) {
    double delta = targetMeters - extensorPositionMeters;

    if (Math.abs(delta) < HighAltitudeConstants.EXTENSOR_ARRIVE_OFFSET) {
      extensorMotors.setAll(0);
      return true;
    }

    // double power = delta / (HighAltitudeConstants.EXTENSOR_BRAKING_METERS *
    // maxPower * maxPower);
    double power = delta / HighAltitudeConstants.EXTENSOR_BRAKING_METERS;
    power = Math.clamp(power, -1, 1) * maxPower;

    extensorMotors.setAll(power);
    return false;
  }

  public void resetEncoders() {
    extensorMotors.resetEncoder();
  }

  @Override
  public void periodic() {
    extensorEncoderPosition = extensorMotors.getEncoderPosition();
    extensorPositionMeters = extensorEncoderPosition *
        HighAltitudeConstants.EXTENSOR_METERS_PER_PULSE;

    Robot.debugNumberSmartDashboard("Extensor Encoder", extensorEncoderPosition);
    Robot.debugNumberSmartDashboard("Extensor Meters", extensorPositionMeters);
    // Robot.debug("ExtensorPos:" + extensorEncoderPosition + " ExtensorDeg: " +
    // extensorPositionDegrees);
  }
}

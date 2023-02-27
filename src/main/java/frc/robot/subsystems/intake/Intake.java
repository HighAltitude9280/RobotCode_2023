// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Intake extends SubsystemBase {
  HighAltitudeMotorGroup intakeMotors;
  DoubleSolenoid intakeSolenoid;

  IntakePosition currentPosition;

  public enum IntakePosition {
    LOWERED, STORED
  }

  /** Creates a new Intake. */
  public Intake() {

    intakeMotors = new HighAltitudeMotorGroup(RobotMap.INTAKE_MOTOR_PORTS, RobotMap.INTAKE_INVERTED_MOTORS_PORTS,
        RobotMap.INTAKE_MOTOR_TYPES);
    intakeMotors.setEncoderInverted(RobotMap.INTAKE_ENCODER_IS_INVERTED);
    intakeMotors.setBrakeMode(HighAltitudeConstants.INTAKE_MOTORS_BRAKING_MODE);

    intakeSolenoid = new DoubleSolenoid(RobotMap.REVPH_MODULE_ID, RobotMap.INTAKE_SOLENOID_MODULE_TYPE,
        RobotMap.INTAKE_SOLENOID_PORTS[0], RobotMap.INTAKE_SOLENOID_PORTS[1]);
    setIntakePosition(HighAltitudeConstants.INTAKE_INITIAL_POSITION);

  }

  public void driveIntake(double speed) {
    intakeMotors.setAll(speed);
  }

  public void stopIntake() {
    intakeMotors.setAll(0);
  }

  public void setIntakePosition(IntakePosition position) {
    if (position == IntakePosition.LOWERED)
      intakeSolenoid.set(RobotMap.INTAKE_LOWERED_VALUE);
    else
      intakeSolenoid.set(RobotMap.INTAKE_STORED_VALUE);
    currentPosition = position;
  }

  public void toggleIntakePosition() {
    if (currentPosition == IntakePosition.LOWERED)
      setIntakePosition(IntakePosition.STORED);
    else
      setIntakePosition(IntakePosition.LOWERED);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

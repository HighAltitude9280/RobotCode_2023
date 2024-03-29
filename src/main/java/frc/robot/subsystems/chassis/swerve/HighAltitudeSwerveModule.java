// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;
import frc.robot.resources.math.Math;
import frc.robot.resources.components.speedController.HighAltitudeMotorController;
import frc.robot.resources.components.speedController.HighAltitudeMotorController.TypeOfMotor;

/** Add your docs here. */
public class HighAltitudeSwerveModule {
    private HighAltitudeMotorController driveMotor;
    private HighAltitudeMotorController directionMotor;

    private boolean isDriveEncoderReversed;
    private boolean isDirectionEncoderReversed;
    private boolean isTalonEncoderReversed;

    private PIDController directionPIDController;

    private TalonSRX absoluteEncoderController;
    private double encoderOffsetPulses;

    public HighAltitudeSwerveModule(int driveMotorPort, TypeOfMotor driveTypeOfMotor, boolean isDriveMotorReversed,
            boolean isDriveEncoderReversed,
            int directionMotorPort, TypeOfMotor directionTypeOfMotor,
            boolean isDirectionMotorReversed, boolean isDirectionEncoderReversed, int encodedTalonPort,
            double encoderOffsetPulses, boolean isTalonEncoderReversed) {

        driveMotor = new HighAltitudeMotorController(driveMotorPort, driveTypeOfMotor);
        driveMotor.setInverted(isDriveMotorReversed);
        driveMotor.setBrakeMode(false);
        this.isDriveEncoderReversed = isDriveEncoderReversed;

        directionMotor = new HighAltitudeMotorController(directionMotorPort, directionTypeOfMotor);
        directionMotor.setInverted(isDirectionMotorReversed);
        directionMotor.setBrakeMode(false);
        this.isDirectionEncoderReversed = isDirectionEncoderReversed;

        directionPIDController = new PIDController(HighAltitudeConstants.SWERVE_DIRECTION_KP, 0.025, 0.0);
        directionPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoderController = new TalonSRX(encodedTalonPort);
        absoluteEncoderController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        if (absoluteEncoderController.configFeedbackNotContinuous(true, 100).value == 0) {
            System.out.println("PulseWidthEncoder not configured to not continuous, you might want to check that...");
        }
        this.isTalonEncoderReversed = isTalonEncoderReversed;
        this.encoderOffsetPulses = encoderOffsetPulses;
        resetEncoders();
    }

    public double getAbsoluteEncoderRad() {
        double angleRadians = absoluteEncoderController.getSelectedSensorPosition()
                * HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE;
        // System.out.print("b4: " + angleRadians);
        angleRadians -= encoderOffsetPulses * HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE;
        return angleRadians * (isTalonEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        recalculateWheelDirection();
    }

    public void recalculateWheelDirection() {
        directionMotor
                .setEncoderPosition((int) ((isDirectionEncoderReversed ? -1.0 : 1.0) * (getAbsoluteEncoderRad()
                        / (HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_PULSE))));

    }

    // Getters for encoder values and velocities

    public double getDriveEncoder() {
        return driveMotor.getEncPosition() * (isDriveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveDistance() {
        return driveMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_PULSE
                * (isDriveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveVelocity() {
        return driveMotor.getEncVelocity() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS
                * (isDriveEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirectionEncoder() {
        return directionMotor.getEncPosition() * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirection() {
        return directionMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_PULSE
                * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    public double getDirectionVelocity() {
        return directionMotor.getEncVelocity()
                * HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_SEC_PER_VELOCITY_UNITS
                * (isDirectionEncoderReversed ? -1.0 : 1.0);
    }

    // Getters for the motor objects themselves

    public HighAltitudeMotorController getDriveMotor() {
        return driveMotor;
    }

    public HighAltitudeMotorController getDirectionMotor() {
        return directionMotor;
    }

    // Getters for the position and state of the module

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getDirection()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getDirection()));
    }

    // STATE SETTER

    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
        directionMotor.set(directionPIDController.calculate(getDirection(), state.angle.getRadians()));
    }

    public void setStateRegardlessOfSpeed(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / HighAltitudeConstants.SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
        directionMotor.set(directionPIDController.calculate(getDirection(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        directionMotor.set(0);
    }

    // Smartdashboard prints for debugging.

    public void putRawEncoderValues(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveEncPos", driveMotor.getEncPosition());
        // SmartDashboard.putNumber(identifier + "DriveEncVel",
        // driveMotor.getEncVelocity());
        SmartDashboard.putNumber(identifier + "DirEncPos", directionMotor.getEncPosition());
        // SmartDashboard.putNumber(identifier + "DirEncVel",
        // directionMotor.getEncVelocity());
        SmartDashboard.putNumber(identifier + "AbsEncPos", absoluteEncoderController.getSelectedSensorPosition());
    }

    public void putAbsEncPos(String identifier) {
        SmartDashboard.putNumber(identifier + "AbsEncPos", absoluteEncoderController.getSelectedSensorPosition());

    }

    public void putProcessedValues(String identifier) {
        SmartDashboard.putNumber(identifier + "DrivePos", getDriveDistance());
        // SmartDashboard.putNumber(identifier + "DriveVel", getDriveVelocity());
        SmartDashboard.putNumber(identifier + "DirPos", getDirection());
        // SmartDashboard.putNumber(identifier + "DirVel", getDirectionVelocity());
        SmartDashboard.putNumber(identifier + "AbsPos", getAbsoluteEncoderRad());
    }

    public void putMotorOutputs(String identifier) {
        SmartDashboard.putNumber(identifier + "DriveOut", driveMotor.getOutput());
        SmartDashboard.putNumber(identifier + "DirOut", directionMotor.getOutput());
    }
}

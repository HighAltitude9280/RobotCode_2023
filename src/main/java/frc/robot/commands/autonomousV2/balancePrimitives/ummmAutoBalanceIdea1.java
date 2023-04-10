// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2.balancePrimitives;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;

public class ummmAutoBalanceIdea1 extends CommandBase {
  SwerveDriveTrain swerveDriveTrain;

  ChassisSpeeds chassisSpeeds;

  double currentYaw;
  double currentRoll;
  double currentRollVel;

  // Determined by whetherr the roll has gone up a certain threshold.
  boolean hasGoneOnChargingStation = false;
  double upOnChargingStationTimeStamp;
  double howManySecondsShouldPassBeforeVelocityIsTakenIntoAccount = 1.0;

  // If the Roll is equal to 0
  boolean balanced = false;

  double k_yawGain = 0.01;

  /** Creates a new SwerveAutoBalanceV1. */
  public ummmAutoBalanceIdea1() {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateAngles();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateAngles();
    balanced = Math.abs(currentRoll) < 4.5;

    // If it hasn't already gone on the charge station, drive forward until angle
    // change

    if (!hasGoneOnChargingStation && balanced) {
      chassisSpeeds = new ChassisSpeeds(0.5, 0, (0 - currentYaw) * k_yawGain);
    }

    if (!hasGoneOnChargingStation && Math.abs(currentRoll) > HighAltitudeConstants.BALANCING_ANGLE_THRESHOLD) {
      hasGoneOnChargingStation = true;
      upOnChargingStationTimeStamp = Timer.getFPGATimestamp();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateAngles() {
    currentYaw = swerveDriveTrain.getPose().getRotation().getDegrees();
    currentRoll = Robot.getRobotContainer().getNavx().getRoll();
    currentRollVel = Robot.getRobotContainer().getNavx().getYVel();
  }
}

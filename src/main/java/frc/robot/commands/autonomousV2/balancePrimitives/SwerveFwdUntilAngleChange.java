// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2.balancePrimitives;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;

public class SwerveFwdUntilAngleChange extends CommandBase {
  SwerveDriveTrain swerveDriveTrain;

  ChassisSpeeds chassisSpeeds;

  double currentRoll;
  double currentYawFromOdometry;

  double kP_Angular = 1.0 / 95.0;
  double vx;
  double kAngleThreshold = HighAltitudeConstants.BALANCING_ANGLE_THRESHOLD;

  /** Creates a new SwerveFwdUntilAngleChange. */
  public SwerveFwdUntilAngleChange(double vx, double kAngleThreshold) {
    this.vx = vx;
    this.kAngleThreshold = kAngleThreshold;
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new SwerveFwdUntilAngleChange. */
  public SwerveFwdUntilAngleChange(double vx) {
    this.vx = vx;
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateAngles();
    double yawError = 0 - currentYawFromOdometry;
    double chassisAngVel = yawError * kP_Angular;

    chassisSpeeds = new ChassisSpeeds(vx, 0, chassisAngVel);
    SwerveModuleState[] moduleStates = HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerveDriveTrain.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(currentRoll) > kAngleThreshold);
  }

  void updateAngles() {
    currentRoll = Robot.getRobotContainer().getNavx().getRoll();
    currentYawFromOdometry = swerveDriveTrain.getPose().getRotation().getDegrees();
  }
}

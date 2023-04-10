// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;
import frc.robot.resources.math.Math;

public class SwerveDriveDistanceFwd extends CommandBase {
  SwerveDriveTrain swerveDriveTrain;
  double targetDistance;
  boolean usingOdometry;
  double brakingDistance = -1;
  double vx;
  double initialDistance;

  double error;

  /** Creates a new DriveSwerveDistance. */
  public SwerveDriveDistanceFwd(double vx, double targetDistance, boolean usingOdometry) {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);

    this.vx = vx;
    this.targetDistance = targetDistance;
    this.usingOdometry = usingOdometry;
  }

  public SwerveDriveDistanceFwd(double vx, double targetDistance, double brakingDistance, boolean usingOdometry) {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);

    this.vx = vx;
    this.targetDistance = targetDistance;
    this.brakingDistance = Math.abs(brakingDistance);
    this.usingOdometry = usingOdometry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (usingOdometry) {
      initialDistance = swerveDriveTrain.getPose().getX();
    } else {
      initialDistance = swerveDriveTrain.getFrontLeft().getDriveDistance();
    }
    updateError();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateError();
    double appliedVel;
    appliedVel = vx;

    if (brakingDistance > 0)
      appliedVel = Math.clamp(error * (1 / brakingDistance), -1.0, 1.0) * vx;

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(appliedVel, 0.0, 0.0);

    // 5. Set the states to the swerve modules
    SwerveModuleState[] moduleStates = HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerveDriveTrain.setModuleStates(moduleStates);

    SmartDashboard.putNumber("erreo", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.stopModules();
  }

  void updateError() {
    if (usingOdometry)
      error = targetDistance - (swerveDriveTrain.getPose().getX() - initialDistance);
    else
      error = targetDistance - (swerveDriveTrain.getFrontLeft().getDriveDistance() - initialDistance);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 0.1;
  }
}

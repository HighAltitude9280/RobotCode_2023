// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.swerve.balance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;

public class SwerveWaitForFall extends CommandBase {
  SwerveDriveTrain swerveDriveTrain;

  ChassisSpeeds chassisSpeeds;

  double currentRollVel;
  double currentYawFromOdometry;

  double vx;

  /** Creates a new SwerveWaitForFall. */
  public SwerveWaitForFall(double vx) {
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

    chassisSpeeds = new ChassisSpeeds(vx, 0, 0);
    SwerveModuleState[] moduleStates = HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerveDriveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.setModulesInXPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(currentRollVel) > HighAltitudeConstants.BALANCING_SPEED_THRESHOLD;
  }

  void updateAngles() {
    currentRollVel = Robot.getRobotContainer().getNavx().getYVel();
    currentYawFromOdometry = swerveDriveTrain.getPose().getRotation().getDegrees();
  }
}

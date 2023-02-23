// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.follower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain.DrivingMode;

public class FollowLimeLightTarget extends CommandBase {
  DrivingMode prevMode;
  double maxPower;

  /** Creates a new FollowLimeLightTarget. */
  public FollowLimeLightTarget() {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevMode = Robot.getRobotContainer().getDriveTrain().getCurrentDrivingMode();
    Robot.getRobotContainer().getDriveTrain().setDrivingMode(DrivingMode.Mecanum);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxPower = 0.5;
    double xPower = (Robot.getRobotContainer().getLimeLightVision().getTx() / 28) * maxPower;
    Robot.getRobotContainer().getDriveTrain().defaultDrive(xPower, 0, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getDriveTrain().setDrivingMode(prevMode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

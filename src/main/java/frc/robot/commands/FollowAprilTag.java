// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class FollowAprilTag extends CommandBase {
  DriveTrain driveTrain;

  /** Creates a new FollowAprilTag. */
  public FollowAprilTag() {
    driveTrain = Robot.getRobotContainer().getDriveTrain();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double position = Robot.getRobotContainer().getVision().getCenterX();
    double dist = Robot.getRobotContainer().getVision().getDist();

    Robot.debugPrint("dist: " + dist);

    double centeredPos = position - 320;

    driveTrain.follow(centeredPos, dist);
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
}
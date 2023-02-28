// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.follower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class FollowTargetJolt extends CommandBase {
  DriveTrain driveTrain;
  double maxPower;

  /* Creates a new FollowTargetJolt. */
  public FollowTargetJolt() {
    driveTrain = Robot.getRobotContainer().getDriveTrain();
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxPower = HighAltitudeConstants.DRIVETRAIN_ALIGN_MAX_SPEED;
    double speed = OI.getInstance().getDefaultDriveY();
    double xPower = (Robot.getRobotContainer().getLimeLightVision().getTx() / 28) * maxPower;
    driveTrain.defaultDrive(xPower, speed, 0, 0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class ForwardUntilAngleChange extends CommandBase {
  DriveTrain driveTrain;

  /** Creates a new ForwardUntilAngleChange. */
  public ForwardUntilAngleChange() {
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
    driveTrain.arcadeDrive(0.375, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Robot.getRobotContainer().getNavx().getRoll()) > HighAltitudeConstants.BALANCING_ANGLE_THRESHOLD;
  }
}

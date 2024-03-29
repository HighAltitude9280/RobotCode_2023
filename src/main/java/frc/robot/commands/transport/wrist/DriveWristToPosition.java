// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Wrist;

public class DriveWristToPosition extends CommandBase {
  Wrist wrist;
  double targetDegrees, maxPower;

  /** Creates a new DriveWristToPosition. */
  public DriveWristToPosition(double targetDegrees, double maxPower) {
    wrist = Robot.getRobotContainer().getWrist();
    addRequirements(wrist);
    this.targetDegrees = targetDegrees;
    this.maxPower = maxPower;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.driveWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.moveTo(targetDegrees, maxPower);
  }
}

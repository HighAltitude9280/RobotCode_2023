// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.extensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Extensor;

public class DriveExtensorToPosition extends CommandBase {
  Extensor extensor;
  double targetDegrees, maxPower;

  /** Creates a new DriveExtensorToPosition. */
  public DriveExtensorToPosition(double taretMeters, double maxPower) {
    extensor = Robot.getRobotContainer().getExtensor();
    addRequirements(extensor);
    this.targetDegrees = taretMeters;
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
    extensor.driveExtensor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extensor.moveTo(targetDegrees, maxPower);
  }
}

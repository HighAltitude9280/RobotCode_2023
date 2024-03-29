// Copyright (c) FIRST and other WPILib contributors.DriveExtensor
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.extensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Extensor;

public class DriveExtensor extends CommandBase {
  Extensor extensor;

  /** Creates a new DriveWrist. */
  public DriveExtensor() {
    extensor = Robot.getRobotContainer().getExtensor();
    addRequirements(extensor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Robot.getRobotContainer().getCurrentGamePieceMode() ==
    // GamePieceMode.MANUAL)
    extensor.driveExtensor(OI.getInstance().getExtensorInput());
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

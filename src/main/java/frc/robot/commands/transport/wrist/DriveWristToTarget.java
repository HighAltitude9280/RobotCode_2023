// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.transport.Wrist;
import frc.robot.commands.transport.TransportTargets.TransportTarget;

public class DriveWristToTarget extends CommandBase {
  Wrist wrist;
  TransportTarget target;
  double wristTarget, maxPower;
  GamePieceMode initialGamePieceMode;

  /** Creates a new DriveWristToPosition. */
  public DriveWristToTarget(TransportTarget target, double maxPower) {
    wrist = Robot.getRobotContainer().getWrist();
    addRequirements(wrist);
    this.target = target;
    this.maxPower = maxPower;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();
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
    // if the GamePieceMode changed, stop the command
    if (Robot.getRobotContainer().getCurrentGamePieceMode() != initialGamePieceMode) {
      return true;
    }
    // if the GamePieceMode is manual or null, stop the command
    switch (initialGamePieceMode) {
      case CONE:
        wristTarget = target.getWristTargetCone();
        break;
      case CUBE:
        wristTarget = target.getWristTargetCube();
        break;
      case MANUAL:
        return true;
      default:
        return true;
    }
    // in any other case, execute the comand normally
    return wrist.moveTo(wristTarget, maxPower);
  }
}

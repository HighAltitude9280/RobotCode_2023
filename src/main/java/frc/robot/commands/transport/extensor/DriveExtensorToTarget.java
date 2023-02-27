// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.extensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.transport.Extensor;
import frc.robot.commands.transport.TransportTargets.TransportTarget;

public class DriveExtensorToTarget extends CommandBase {
  Extensor extensor;
  TransportTarget target;
  double extensorTarget, maxPower;
  GamePieceMode initialGamePieceMode;

  /** Creates a new DriveExtensorToPosition. */
  public DriveExtensorToTarget(TransportTarget target, double maxPower) {
    extensor = Robot.getRobotContainer().getExtensor();
    addRequirements(extensor);
    this.target = target;
    this.maxPower = maxPower;
    initialGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.debugPrint("Driving extensor to a target.");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extensor.driveExtensor(0);
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
        extensorTarget = target.getExtensorTargetCone();
        break;
      case CUBE:
        extensorTarget = target.getExtensorTargetCube();
        break;
      case MANUAL:
        return true;
      default:
        return true;
    }
    // in any other case, execute the comand normally
    return extensor.moveTo(extensorTarget, maxPower);
  }
}

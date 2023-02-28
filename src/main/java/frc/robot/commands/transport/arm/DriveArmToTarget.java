// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.transport.Arm;
import frc.robot.commands.transport.TransportTargets.TransportTarget;

public class DriveArmToTarget extends CommandBase {
  Arm arm;
  TransportTarget target;
  double armTarget, maxPower;
  GamePieceMode initialGamePieceMode;

  /** Creates a new DriveArmToPosition. */
  public DriveArmToTarget(TransportTarget target, double maxPower) {
    arm = Robot.getRobotContainer().getArm();
    addRequirements(arm);
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
    Robot.debugPrint("Driving arm to a target.");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.driveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the GamePieceMode changed, stop the command
    if (Robot.getRobotContainer().getCurrentGamePieceMode() != initialGamePieceMode) {
      return true;
    }

    switch (initialGamePieceMode) {
      case CONE:
        armTarget = target.getArmTargetCone();
        break;
      case CUBE:
        armTarget = target.getArmTargetCube();
        break;
      case MANUAL:
        return true;
      default:
        return true;
    }

    return arm.moveTo(armTarget, maxPower);
  }
}

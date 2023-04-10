// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pieceHandlers.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.gripper.Gripper;

public class GripperInDontHold extends CommandBase {
  Gripper gripper;

  /** Creates a new GripperInDontHold. */
  public GripperInDontHold() {
    gripper = Robot.getRobotContainer().getGripper();
    addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GamePieceMode currentMode = Robot.getRobotContainer().getCurrentGamePieceMode();
    switch (currentMode) {
      case CONE:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_CONE_IN_SPEED);
        break;
      case CUBE:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_CUBE_IN_SPEED);
        break;
      case MANUAL:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_DEFAULT_IN_SPEED);
        break;
      default:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_DEFAULT_IN_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

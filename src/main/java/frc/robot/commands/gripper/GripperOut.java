// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.gripper.Gripper;

public class GripperOut extends CommandBase {
  Gripper gripper;

  /** Creates a new GripperGrab. */
  public GripperOut() {
    gripper = Robot.getRobotContainer().getGripper();
    addRequirements(gripper);
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
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_CONE_OUT_SPEED);
        break;
      case CUBE:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_CUBE_OUT_SPEED);
        break;
      case MANUAL:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_DEFAULT_OUT_SPEED);
        break;
      default:
        gripper.driveGripper(HighAltitudeConstants.GRIPPER_DEFAULT_OUT_SPEED);
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

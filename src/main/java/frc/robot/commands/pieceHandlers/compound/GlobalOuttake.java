// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pieceHandlers.compound;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;

public class GlobalOuttake extends CommandBase {
  Intake intake;
  Gripper gripper;
  GamePieceMode currentGamePieceMode;
  double gripperSpeed;

  /** Creates a new GlobalOuttake. */
  public GlobalOuttake() {
    intake = Robot.getRobotContainer().getIntake();
    gripper = Robot.getRobotContainer().getGripper();
    addRequirements(intake, gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();

    switch (currentGamePieceMode) {
      case CONE:
        gripperSpeed = HighAltitudeConstants.GRIPPER_CONE_OUT_SPEED;
        break;
      case CUBE:
        gripperSpeed = HighAltitudeConstants.GRIPPER_CUBE_OUT_SPEED;
        break;
      case MANUAL:
      default:
        gripperSpeed = HighAltitudeConstants.GRIPPER_DEFAULT_OUT_SPEED;
    }
    gripper.driveGripper(gripperSpeed);
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

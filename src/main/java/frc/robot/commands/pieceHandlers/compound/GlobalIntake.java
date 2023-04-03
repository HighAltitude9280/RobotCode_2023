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
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GlobalIntake extends CommandBase {
  Intake intake;
  Gripper gripper;

  GamePieceMode currentGamePieceMode;
  IntakePosition intakePosition;
  double gripperSpeed;

  /** Creates a new GlobalIntake. */
  public GlobalIntake() {
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
    intakePosition = intake.getCurrentPosition();
    currentGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();

    switch (currentGamePieceMode) {
      case CONE:
        gripperSpeed = HighAltitudeConstants.GRIPPER_CONE_IN_SPEED;
        break;
      case CUBE:
        gripperSpeed = HighAltitudeConstants.GRIPPER_CUBE_IN_SPEED;
        break;
      case MANUAL:
      default:
        gripperSpeed = HighAltitudeConstants.GRIPPER_DEFAULT_IN_SPEED;
    }
    gripper.driveGripper(gripperSpeed);

    switch (intakePosition) {
      case STORED:
        intake.driveIntake(HighAltitudeConstants.INTAKE_OUT_SPEED);
        break;
      case LOWERED:
        intake.driveIntake(HighAltitudeConstants.INTAKE_IN_SPEED);
        break;
      default:
        intake.driveIntake(HighAltitudeConstants.INTAKE_IN_SPEED);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

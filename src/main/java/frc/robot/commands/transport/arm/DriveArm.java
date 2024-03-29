// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.transport.Arm;

public class DriveArm extends CommandBase {
  Arm arm;

  /** Creates a new DriveArm. */
  public DriveArm() {
    arm = Robot.getRobotContainer().getArm();
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.getRobotContainer().getCurrentGamePieceMode() == GamePieceMode.MANUAL
        || Robot.getRobotContainer().getCurrentGamePieceMode() == GamePieceMode.CONE)
      arm.driveArm(OI.getInstance().getArmInput());
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

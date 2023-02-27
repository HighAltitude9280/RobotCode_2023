// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class SetIntakePosition extends CommandBase {
  Intake intake;
  IntakePosition position;

  /** Creates a new SetIntakePosition. */
  public SetIntakePosition(IntakePosition position) {
    this.position = position;
    intake = Robot.getRobotContainer().getIntake();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

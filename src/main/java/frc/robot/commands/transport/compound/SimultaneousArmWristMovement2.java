// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Wrist;

public class SimultaneousArmWristMovement2 extends CommandBase {
  Arm arm;
  Wrist wrist;
  double delta;

  /** Creates a new SimultaneousArmWristMovement2. */
  public SimultaneousArmWristMovement2() {
    arm = Robot.getRobotContainer().getArm();
    wrist = Robot.getRobotContainer().getWrist();
    addRequirements(arm, wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delta = arm.getCurrentAngle() - wrist.getCurrentAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = OI.getInstance().getArmInput();
    double wristSpeed = OI.getInstance().getWristInput();

    arm.driveArm(armSpeed);

    if (wristSpeed != 0) {
      delta = arm.getCurrentAngle() - wrist.getCurrentAngle();
      wrist.driveWrist(wristSpeed);
    } else {
      wrist.moveTo(arm.getCurrentAngle() - delta, armSpeed * 0.5);
    }
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

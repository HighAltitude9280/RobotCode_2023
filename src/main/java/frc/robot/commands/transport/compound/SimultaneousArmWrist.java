// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Wrist;

/*
 * Funcionó pero no tanto, luego vale la pena explorarlo jaiksdjfawejifpaoie
 */
public class SimultaneousArmWrist extends CommandBase {
  Arm arm;
  Wrist wrist;

  double initialWristAngle;
  double delta;

  /** Creates a new SimultaneousArmWrist. */
  public SimultaneousArmWrist() {
    arm = Robot.getRobotContainer().getArm();
    wrist = Robot.getRobotContainer().getWrist();
    addRequirements(arm, wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialWristAngle = wrist.getCurrentAngle();
    delta = arm.getCurrentAngle() - wrist.getCurrentAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double speed = OI.getInstance().getCopilot().getAxis(AxisType.RIGHT_Y) *
    // 0.25;
    double speed = OI.getInstance().getArmInput();
    delta = arm.getCurrentAngle() - wrist.getCurrentAngle();

    arm.driveArm(speed);
    wrist.moveTo(initialWristAngle - delta, speed * 0.625);
    System.out.println("Wrist a target: " + (initialWristAngle - delta) + "Con velocidad: " + speed * 0.625 + "Init: "
        + initialWristAngle + "Delta: " + delta);
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

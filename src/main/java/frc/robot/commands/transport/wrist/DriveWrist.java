// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.subsystems.transport.Wrist;

public class DriveWrist extends CommandBase {
  Wrist wrist;

  /** Creates a new DriveWrist. */
  public DriveWrist() {
    wrist = Robot.getRobotContainer().getWrist();
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.driveWrist(OI.getInstance().getPilot().getAxis(AxisType.RIGHT_Y));
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

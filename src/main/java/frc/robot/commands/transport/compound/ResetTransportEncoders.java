// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.resources.components.PWMLEDStrip.commands.compound.FlashColor;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Wrist;
import frc.robot.subsystems.transport.Extensor;

public class ResetTransportEncoders extends CommandBase {
  Wrist wrist;
  Arm arm;
  Extensor extensor;

  /** Creates a new ResetTransportEncoders. */
  public ResetTransportEncoders() {
    wrist = Robot.getRobotContainer().getWrist();
    arm = Robot.getRobotContainer().getArm();
    extensor = Robot.getRobotContainer().getExtensor();

    addRequirements(wrist, arm, extensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.resetEncoders();
    arm.resetEncoders();
    extensor.resetEncoders();
    System.out.println("Transport encoders reset");
    CommandScheduler.getInstance().schedule(new FlashColor(0, 0, 255, 0.125));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

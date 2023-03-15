// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.robotParameters;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.transport.arm.DriveArm;
import frc.robot.commands.transport.compound.SimultaneousArmWristMovement2;
import frc.robot.commands.transport.wrist.DriveWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetGamePieceMode extends InstantCommand {
  RobotContainer robotContainer;
  GamePieceMode mode;

  public SetGamePieceMode(GamePieceMode mode) {
    robotContainer = Robot.getRobotContainer();
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotContainer.setCurrentGamePieceMode(mode);
    if (mode.equals(GamePieceMode.CONE) || mode.equals(GamePieceMode.CUBE)) {
      robotContainer.getArm().getCurrentCommand().cancel();
      robotContainer.getWrist().getCurrentCommand().cancel();
      robotContainer.getArm().setDefaultCommand(new SimultaneousArmWristMovement2());
      robotContainer.getWrist().setDefaultCommand(new SimultaneousArmWristMovement2());
    } else {
      robotContainer.getArm().getCurrentCommand().cancel();
      robotContainer.getWrist().getCurrentCommand().cancel();
      robotContainer.getArm().setDefaultCommand(new DriveArm());
      robotContainer.getWrist().setDefaultCommand(new DriveWrist());
    }

  }
}

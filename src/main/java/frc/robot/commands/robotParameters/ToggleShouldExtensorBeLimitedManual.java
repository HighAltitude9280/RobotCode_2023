// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.robotParameters;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.resources.components.PWMLEDStrip.commands.compound.FlashColor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleShouldExtensorBeLimitedManual extends InstantCommand {
  public ToggleShouldExtensorBeLimitedManual() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Robot.getRobotContainer().getCurrentGamePieceMode() == GamePieceMode.MANUAL) {
      Robot.getRobotContainer()
          .setShouldExtensorBeLimitedManual(
              !Robot.getRobotContainer().getShouldExtensorBeSlowerInManual());
      if (Robot.getRobotContainer().getShouldExtensorBeSlowerInManual())
        CommandScheduler.getInstance().schedule(new FlashColor(255, 0, 0, 0.125));
      else
        CommandScheduler.getInstance().schedule(new FlashColor(0, 255, 0, 0.125));
    }
  }
}

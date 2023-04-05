// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.PWMLEDStrip.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.resources.math.Math;

public class DisplayGamePieceModeVariableIntensity extends CommandBase {
  Supplier<GamePieceMode> a;
  Supplier<Double> speed, strafe, turn;
  int hue;

  /** Creates a new DisplayGamePieceMode. */
  public DisplayGamePieceModeVariableIntensity() {
    addRequirements(Robot.getRobotContainer().getLeds());
    a = () -> Robot.getRobotContainer().getCurrentGamePieceMode();
    hue = 64;

    speed = () -> OI.getInstance().getDefaultSwerveDriveSpeed();
    strafe = () -> OI.getInstance().getDefaultSwerveDriveStrafe();
    turn = () -> OI.getInstance().getDefaultSwerveDriveTurn();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (a.get().equals(GamePieceMode.MANUAL))
      hue = HighAltitudeConstants.LEDS_9280_HUE;
    else if (a.get().equals(GamePieceMode.CUBE))
      hue = HighAltitudeConstants.LEDS_CUBE_HUE;
    else if (a.get().equals(GamePieceMode.CONE))
      hue = HighAltitudeConstants.LEDS_CONE_HUE;

    double input = Math.clamp(Math.abs(speed.get()) + Math.abs(strafe.get()) + Math.abs(turn.get()), 0.0, 1.0);
    Robot.getRobotContainer().getLeds().setCoolerFireAnimationWithInput(hue, input, 0.0, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getLeds().allLedsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

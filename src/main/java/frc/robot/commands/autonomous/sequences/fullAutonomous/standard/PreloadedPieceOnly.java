// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.autonomous.primitives.transport.BreakInitialConfig;
import frc.robot.commands.pieceHandlers.compound.GlobalOuttake;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.TransportGoTo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreloadedPieceOnly extends SequentialCommandGroup {
  GamePieceMode gamePieceMode;

  /** Creates a new PreloadedPieceOnly. */
  public PreloadedPieceOnly(GamePieceMode gamePieceMode) {
    this.gamePieceMode = gamePieceMode;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetGamePieceMode(gamePieceMode),
        new BreakInitialConfig(),
        new TransportGoTo(TransportTarget.TOP_ROW_BACK),
        new GlobalOuttake().withTimeout(0.75));
  }
}

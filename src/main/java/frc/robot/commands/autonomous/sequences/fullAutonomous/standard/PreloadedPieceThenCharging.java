// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.transport.BreakInitialConfig;
import frc.robot.commands.pieceHandlers.compound.GlobalOuttake;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.WristArmGoTo;
import frc.robot.commands.transport.wrist.DriveWristToPosition;

public class PreloadedPieceThenCharging extends SequentialCommandGroup {
  GamePieceMode gamePieceMode;

  /**
   * Places pre-loaded cone, then moves forward 2.5m
   * 
   */
  public PreloadedPieceThenCharging(GamePieceMode gamePieceMode) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.gamePieceMode = gamePieceMode;
    addCommands(
        new SetGamePieceMode(gamePieceMode),
        new BreakInitialConfig(),
        // Place pre-loaded cone.
        new WristArmGoTo(TransportTarget.TOP_ROW),
        new GlobalOuttake().withTimeout(0.75),
        Commands.parallel(
            Commands.sequence(
                new DriveWristToPosition(290.0, HighAltitudeConstants.WRIST_AUTO_MAX_POWER),
                new WristArmGoTo(TransportTarget.RESTING)),
            new MoveStraight(0.4, 0.25))
    // ,new ForwardUntilAngleChange(),
    // new AutoBalance()
    );
  }
}

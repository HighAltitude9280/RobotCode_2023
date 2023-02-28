// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.autonomous.primitives.AutoBalance;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.transport.BreakInitialConfig;
import frc.robot.commands.pieceHandlers.compound.GlobalOuttake;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.TransportGoTo;

public class preloadedConeThenCharging extends SequentialCommandGroup {

  /**
   * Places pre-loaded cone, then moves forward 2.5m
   * 
   */
  public preloadedConeThenCharging() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // TODO: test autonomous. Remember to first test BreakInitialConfig()
    addCommands(
        new SetGamePieceMode(GamePieceMode.CONE),
        new BreakInitialConfig(),
        // Place pre-loaded cone.
        new TransportGoTo(TransportTarget.TOP_ROW),
        new GlobalOuttake().withTimeout(0.75),
        new MoveStraight(4, 0.7),
        new MoveStraight(-1.8, 0.5),
        new AutoBalance());
  }
}

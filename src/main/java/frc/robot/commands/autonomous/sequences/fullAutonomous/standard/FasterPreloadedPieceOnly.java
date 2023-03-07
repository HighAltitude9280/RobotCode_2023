// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.transport.BreakInitialConfig;
import frc.robot.commands.drivetrain.drivingParameters.transmission.DrivetrainSetTransmission;
import frc.robot.commands.pieceHandlers.compound.GlobalOuttake;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.TransportGoTo;
import frc.robot.commands.transport.compound.TransportGoToParallel;
import frc.robot.subsystems.chassis.DriveTrain.TransmissionMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FasterPreloadedPieceOnly extends SequentialCommandGroup {
  GamePieceMode gamePieceMode;

  /** Creates a new FasterPreloadedPieceOnly. */
  public FasterPreloadedPieceOnly(GamePieceMode g) {
    gamePieceMode = g;
    addCommands(
        new SetGamePieceMode(g),
        new DrivetrainSetTransmission(TransmissionMode.torque),
        new BreakInitialConfig(),
        new TransportGoToParallel(TransportTarget.TOP_ROW),
        new GlobalOuttake().withTimeout(0.5));
    // new TransportGoTo(TransportTarget.RESTING)); ESTE SI JALA
  }
}

/*
 * addCommands(
 * new SetGamePieceMode(GamePieceMode.CONE),
 * new DrivetrainSetTransmission(TransmissionMode.torque),
 * new BreakInitialConfig(),
 * new TransportGoToParallel(TransportTarget.TOP_ROW),
 * new GlobalOuttake().withTimeout(0.5),
 * Commands.parallel(
 * new MoveStraight(2.5, 0.5),
 * Commands.sequence(
 * new DriveWristToPosition(290.0, HighAltitudeConstants.WRIST_AUTO_MAX_POWER),
 * new WristArmGoTo(TransportTarget.RESTING))));
 * }
 */
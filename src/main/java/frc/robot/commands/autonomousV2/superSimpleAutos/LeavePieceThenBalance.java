// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2.superSimpleAutos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.autonomousV2.balancePrimitives.AutoBalance2Fwd;
import frc.robot.commands.drivetrain.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.pieceHandlers.gripper.GripperHold;
import frc.robot.commands.pieceHandlers.gripper.GripperInDontHold;
import frc.robot.commands.pieceHandlers.gripper.GripperOut;
import frc.robot.commands.robotParameters.SetGamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.NewTransportGoTo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeavePieceThenBalance extends SequentialCommandGroup {
  /** Creates a new LeavePieceThenBalance. */
  public LeavePieceThenBalance(GamePieceMode gamePieceMode, TransportTarget target) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ResetOdometryZeros(),
        new SetGamePieceMode(gamePieceMode),
        Commands.deadline(new NewTransportGoTo(target),
            new GripperInDontHold().withTimeout(0.5).andThen(new GripperHold())),
        new GripperOut().withTimeout(0.75),
        Commands.parallel(
            new NewTransportGoTo(TransportTarget.RESTING),
            new AutoBalance2Fwd()));
  }
}

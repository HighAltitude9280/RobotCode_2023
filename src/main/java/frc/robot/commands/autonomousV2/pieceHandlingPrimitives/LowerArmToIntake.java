// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2.pieceHandlingPrimitives;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pieceHandlers.gripper.GripperIn;
import frc.robot.commands.pieceHandlers.gripper.GripperInDontHold;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.TransportGoToParallel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowerArmToIntake extends ParallelDeadlineGroup {
  /** Creates a new LowerArmToIntake. */
  public LowerArmToIntake() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
        new TransportGoToParallel(TransportTarget.FLOOR).andThen(new WaitCommand(0.25)));
    addCommands(
        new GripperInDontHold());
    // addCommands(new FooCommand(), new BarCommand());
  }
}

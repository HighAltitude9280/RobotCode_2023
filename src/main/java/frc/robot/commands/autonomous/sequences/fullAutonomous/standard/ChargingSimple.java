// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.primitives.AutoBalanceSimpleFwd;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargingSimple extends SequentialCommandGroup {
  /** Creates a new ChargingSimple. */
  public ChargingSimple() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ForwardUntilAngleChange(),
        new MoveStraight(10, 0.25).withTimeout(2.375),
        new AutoBalanceSimpleFwd(),
        // new MoveStraight(-0.075, 0.08)); // ASI ERA ASÍ ESTABA
        new MoveStraight(-0.11, 0.08));
  }
}

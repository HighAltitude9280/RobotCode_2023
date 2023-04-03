// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.primitives.AutoBalance;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargingAcc extends SequentialCommandGroup {
  /** Creates a new ChargingAcc. */
  public ChargingAcc() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ForwardUntilAngleChange(),
        new MoveStraight(0.25, 0.2),
        new AutoBalance());
  }
}

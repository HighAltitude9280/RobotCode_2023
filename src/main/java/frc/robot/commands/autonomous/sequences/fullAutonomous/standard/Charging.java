// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.standard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.primitives.AutoBalance;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;

public class Charging extends SequentialCommandGroup {

  /** 
   * Places pre-loaded cone, then moves forward 2.5m 
   * 
   * */
  public Charging() 
  {
    addCommands(
      new MoveStraight(1.8, 0.5),
      new AutoBalance()
      );
  }
}

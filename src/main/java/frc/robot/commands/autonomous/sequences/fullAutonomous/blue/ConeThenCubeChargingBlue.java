// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.Paths;
import frc.robot.commands.autonomous.primitives.AutoBalance;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.stepControl.SplineMove;
import frc.robot.commands.drivetrain.drivingSensors.resetOdometry;

public class ConeThenCubeChargingBlue extends SequentialCommandGroup {

  /** 
   * Leaves a cone at position A, intakes cube number 1 and places it at position B,
   * then goes to the charging station and auto balances.
   */
  public ConeThenCubeChargingBlue() {
    addCommands(

      new resetOdometry(1.89, 4.96),
      //Leave preloaded cone at position A
      //Intake in
      new MoveStraight(4.9, 1,3.9),
      //Intake off
      new SplineMove(Paths.piece1ToPositionBBlue, 
        1, true, false, true, true),
      //Place cube
      new SplineMove(Paths.positionBToChargingBlue, 1,
       true, false, false, false),
      new AutoBalance()

    );
  }
}

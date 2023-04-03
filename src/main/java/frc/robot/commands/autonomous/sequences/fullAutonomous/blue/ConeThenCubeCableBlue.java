// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sequences.fullAutonomous.blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.Paths;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.stepControl.SplineMove;
import frc.robot.commands.drivetrain.drivingSensors.ResetOdometry;

public class ConeThenCubeCableBlue extends SequentialCommandGroup {

  /**
   * Leaves a cone at position I, intakes cube number 4 and places it at position
   * H.
   */
  public ConeThenCubeCableBlue() {
    addCommands(

        new ResetOdometry(1.91, 0.50),
        // Leave preloaded cone at position A
        // Intake in
        new MoveStraight(4.68, 1, -4.77),
        // Intake off
        new SplineMove(Paths.piece4ToPositionHBlue,
            1, true, false, true, true)
    // Place cube

    );
  }
}

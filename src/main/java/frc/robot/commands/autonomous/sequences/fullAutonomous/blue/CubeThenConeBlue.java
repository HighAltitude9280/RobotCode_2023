package frc.robot.commands.autonomous.sequences.fullAutonomous.blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.Paths;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.stepControl.SplineMove;
import frc.robot.commands.drivetrain.drivingSensors.ResetOdometry;

public class CubeThenConeBlue extends SequentialCommandGroup {
    /**
     * Leaves pre-loaded cube at position B, intakes cone number 1 and places it at
     * position C.
     */
    public CubeThenConeBlue() {
        addCommands(
                // Place pre-loaded cube
                new ResetOdometry(1.91, 4.42),
                // Start intake
                new MoveStraight(4.68, 1, -2),
                // stop intake
                new SplineMove(Paths.piece1ToPositionCBlue,
                        1, true, false, true, true));
    }
}

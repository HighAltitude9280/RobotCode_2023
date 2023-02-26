package frc.robot.commands.autonomous.sequences.fullAutonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.Paths;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.stepControl.SplineMove;
import frc.robot.commands.drivetrain.drivingSensors.resetOdometry;

public class CubeThenConeRed extends SequentialCommandGroup
{

    public CubeThenConeRed()
    {
        addCommands(
            //Place pre-loaded cube
            new resetOdometry(1.91, 4.42),
            //Start intake
            new MoveStraight(4.68, 1, 2),
            //stop intake
            new SplineMove(Paths.piece1ToPositionCRed, 
                1, true, false, true, true)
        );
    }
}
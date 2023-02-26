package frc.robot.commands.autonomous.sequences.fullAutonomous.red;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.Paths;
import frc.robot.commands.autonomous.primitives.AutoBalance;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.autonomous.primitives.stepControl.SplineMove;
import frc.robot.commands.drivetrain.drivingSensors.resetOdometry;

public class CubeThenConeStationRed extends SequentialCommandGroup
{

    /**
     * Leaves pre-loaded cube at position B, intakes cone number 1 and places it at position C, 
     * then goest to the charging station and autobalances
     */
    public CubeThenConeStationRed()
    {
        addCommands(
            //Place pre-loaded cube
            new resetOdometry(1.91, 3.60),
            //Start intake
            new MoveStraight(4.68, 1,2),
            //stop intake
            new SplineMove(Paths.piece1ToPositionCRed, 
                1, true, false, true, true),
            //place cone,
            new SplineMove(Paths.positionCToChargingRed, 
            1, true, false, false, false),
            new AutoBalance()
        );
    }
}

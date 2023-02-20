package frc.robot.commands.autonomous.sequences.fullAutonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.commands.drivetrain.drivingSensors.resetOdometry;

public class CubeThenConeBlue extends SequentialCommandGroup
{
    public CubeThenConeBlue()
    {
        super(

            new resetOdometry(1.91, 4.42),
            new MoveStraight(4.68, 1,-2)

        );
    }
}

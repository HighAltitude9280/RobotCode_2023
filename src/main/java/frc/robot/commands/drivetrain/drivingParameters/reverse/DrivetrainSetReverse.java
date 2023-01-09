/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.drivingParameters.reverse;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DrivetrainSetReverse extends InstantCommand 
{
    boolean reverse;

    /**
     * Changes the orientation of the robot, when driving in reverse, the back of the 
     * robot will become the front. 
     * 
     * @param reverse True to drive in reverse, false to use default.
     */
    public DrivetrainSetReverse(boolean reverse) 
    {
        this.reverse = reverse;
    }

    @Override
    public void initialize() 
    {
        Robot.getRobotContainer().getDriveTrain().setOrientation(reverse);
    }
}

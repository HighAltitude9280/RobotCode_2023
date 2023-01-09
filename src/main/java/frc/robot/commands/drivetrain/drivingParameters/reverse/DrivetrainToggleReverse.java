/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.drivingParameters.reverse;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DrivetrainToggleReverse extends InstantCommand 
{
    /**
    * Toggles the orientation of the robot (reverse/default).
    */         
    public DrivetrainToggleReverse()
    {
    }

    @Override
    public void initialize() 
    {
        Robot.getRobotContainer().getDriveTrain().changeOrientation();        
    }
}

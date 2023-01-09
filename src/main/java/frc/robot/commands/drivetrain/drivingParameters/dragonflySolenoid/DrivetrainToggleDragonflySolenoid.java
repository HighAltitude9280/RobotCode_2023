/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.drivingParameters.dragonflySolenoid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DrivetrainToggleDragonflySolenoid extends InstantCommand 
{

    /**
     * Toggles the state of the dragonfly module (raised/lowered).
     */
    public DrivetrainToggleDragonflySolenoid() 
    {
    }

    @Override
    public void initialize() 
    {
        Robot.getRobotContainer().getDriveTrain().toggleDragonflySolenoid();
    }
}

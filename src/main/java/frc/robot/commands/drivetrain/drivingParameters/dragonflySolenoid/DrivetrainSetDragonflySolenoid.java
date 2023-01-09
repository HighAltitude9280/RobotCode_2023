/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.drivingParameters.dragonflySolenoid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain.WheelState;
import frc.robot.Robot;

public class DrivetrainSetDragonflySolenoid extends InstantCommand 
{
    WheelState state;

    /**
     * Raises or lowers the dragonfly wheel.
     * 
     * @param state The desired state of the dragonfly module, either raised or lowered. 
     */
    public DrivetrainSetDragonflySolenoid(WheelState state) 
    {
        this.state = state;
    }

    @Override
    public void initialize() 
    {
        Robot.getRobotContainer().getDriveTrain().setDragonflySolenoid(state);
    }
}

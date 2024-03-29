/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.drivingParameters.drivingModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain.DrivingMode;;

public class DrivetrainSetDrivingMode extends InstantCommand {
    DrivingMode mode;

    /**
     * Changes the current driving mode to the desired mode. Note that there are
     * 4 modes of driving:
     * </p>
     * 
     * <ul>
     * 
     * <li>Tank: The default manual driving mode, where speed, turn, and dragonfly
     * power
     * parameters are controlled with the joystick inputs.</li>
     * 
     * <li>Mecanum: Assisted driving mode where the three motor groups are
     * controlled automatically
     * to create a cartesian move relative to the robot.</li>
     * 
     * <li>Swerve: Assited driving mode where the three motor groups are controlled
     * automatically
     * to create a cartesian move relative to the field, regardless of the robot
     * orientation</li>
     * 
     * <li>Pivot: Assisted driving mode where the robot pivots on one side.</li>
     * 
     * </ul>
     * 
     * @param mode The desired driving mode.
     */
    public DrivetrainSetDrivingMode(DrivingMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().setDrivingMode(mode);
    }
}

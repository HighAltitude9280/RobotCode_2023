/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain.drivingParameters.transmission;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain.TransmissionMode;

public class DrivetrainSetTransmission extends InstantCommand {
    TransmissionMode mode;

    /**
     * Sets the transmission to the desired mode (torque or speed).
     * 
     * @param mode The desired transmission mode (torque or speed).
     */
    public DrivetrainSetTransmission(TransmissionMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().setTransmissionState(mode);
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.OI;

public class DefaultDrive extends CommandBase {

    public DefaultDrive() {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double x = OI.getInstance().getPilot().getAxis(AxisType.LEFT_X);
        double y = -OI.getInstance().getPilot().getAxis(AxisType.LEFT_Y);

        double turn = OI.getInstance().getPilot().getAxis(AxisType.RIGHT_X);
        double dragonfly = OI.getInstance().getPilot().getAxis(AxisType.RIGHT_X);

        Robot.getRobotContainer().getDriveTrain().defaultDrive(x, y, turn, dragonfly);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

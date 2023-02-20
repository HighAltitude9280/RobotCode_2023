package frc.robot.commands.drivetrain.drivingSensors;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class resetOdometry extends InstantCommand
{
    double x, y;
    
    /**
     * Resets the drivetrain odometry, setting the current pose 
     * to the given x and y coordinates.
     * 
     * @param x     The new x coordinate for the robot pose.
     * @param y     The new x coordinate for the robot pose.
     */
    public resetOdometry(double x, double y)
    {
        this.x =x;
        this.y=y;
    }    

    @Override 
    public void initialize()
    {
        Robot.getRobotContainer().getDriveTrain().resetOdometry(x, y);
    }
}

package frc.robot.commands.autonomous.primitives.stepControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TurnTo extends CommandBase{

    double target, maxPower;


    /**
     * Turns the robot on its own axis to the specified angle (until the gyroscope reports 
     * the given angle) using step control, then stops. 
     *  
     * @param targetAngle   The desired angle of the robot at the end of the commmand.     
     * @param maxPower      The maximum angular velocity of the robot (from 0 to 1).
     */
    public TurnTo(double targetAngle, double maxPower)
    {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        
        this.target = targetAngle;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Robot.getRobotContainer().getDriveTrain().turn(target, maxPower);
    }
    
    @Override
    public void end(boolean interrupted)
    {
        Robot.getRobotContainer().getDriveTrain().stop();
    }
    
}

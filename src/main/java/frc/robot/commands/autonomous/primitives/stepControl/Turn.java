package frc.robot.commands.autonomous.primitives.stepControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Turn extends CommandBase{

    double degrees, maxPower;

    double target;

    /**
     * Turns the robot on its own axis the specified amount of degrees using step control, 
     * then stops. 
     *  
     * @param degrees   The amount of degrees that the robot will turn.
     * @param maxPower  The maximum angular velocity of the robot (from 0 to 1).
     */
    public Turn(double degrees, double maxPower)
    {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        
        this.degrees = degrees;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize()
    {
        target = Robot.getRobotContainer().getNavx().getYaw() + degrees;
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

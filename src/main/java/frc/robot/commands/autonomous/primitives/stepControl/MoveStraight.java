package frc.robot.commands.autonomous.primitives.stepControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveStraight extends CommandBase{

    double meters, maxPower, angleToKeep;
    boolean keepAngle;

    double target;

    /**
     * Moves the robot in a straight line the specified distance using step control, then stops.
     *  
     * @param meters    The distance that the robot will move (in meters).
     * @param maxPower  The maximum speed of the robot (from 0 to 1).
     */
    public MoveStraight(double meters, double maxPower)
    {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        
        this.meters = meters;
        this.maxPower = maxPower;
        keepAngle = false;
    }

    /**
     * Moves the robot in a straight line the specified distance using step control, while
     * keeping a certain angle to ensure that the robot will follow a straight path.
     * 
     * @param meters        The distance that the robot will move (in meters).
     * @param maxPower      The maximum speed of the robot (from 0 to 1).
     * @param angleToKeep   The angle that the robot will keep (in degrees, from -180 to 180).
     */
    public MoveStraight(double meters, double maxPower, double angleToKeep)
    {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        
        this.meters = meters;
        this.maxPower = maxPower;
        this.angleToKeep = angleToKeep;
        keepAngle = true;
    }

    @Override
    public void initialize()
    {
        target = Robot.getRobotContainer().getDriveTrain().getLeftEncoderDistance() + meters;
    }

    @Override
    public boolean isFinished()
    {
        if(keepAngle)
        {
            return Robot.getRobotContainer().getDriveTrain().moveStraight(target, maxPower, angleToKeep);
        }
        return Robot.getRobotContainer().getDriveTrain().moveStraight(target, maxPower);
    }
    
    @Override
    public void end(boolean interrupted)
    {
        Robot.getRobotContainer().getDriveTrain().stop();
    }
    
}

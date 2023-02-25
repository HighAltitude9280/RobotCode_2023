// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.primitives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.resources.components.Navx;
import frc.robot.resources.math.Math;
import frc.robot.subsystems.chassis.DriveTrain.TransmissionMode;

public class AutoBalance extends CommandBase 
{

  boolean stable, balanced;

  boolean holding;
  double encoderTargetToHold; 

  double lastAngularSpeed; 
  double lastTime;

  Navx navx;

  /** Creates a new AutoBalance. */
  public AutoBalance() 
  {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    checkAngles();
    navx = Robot.getRobotContainer().getNavx();
    Robot.getRobotContainer().getDriveTrain().setTransmissionState(TransmissionMode.torque);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    checkAngles();

    double deltaTo0 = Math.abs(Math.deltaAngle(navx.getYaw(), 0));
    double deltaTo180 = Math.abs(Math.deltaAngle(navx.getYaw(), 180));

    double angleTarget = (deltaTo0 < deltaTo180) ? 0 : 180;

    if(stable && !balanced)
    {
      //Prevents the robot from falling off the charging station
      if(Math.abs(Math.deltaAngle(navx.getYaw(), angleTarget)) > HighAltitudeConstants.BALANCING_ALIGNED_THRESHOLD )
      {
        Robot.getRobotContainer().getDriveTrain().turn(angleTarget, 0.6);
      }
      else
      {
        double direction = navx.getPitch() / Math.abs(navx.getPitch());
        double power = HighAltitudeConstants.BALANCING_DEFAULT_POWER * direction;

        Robot.getRobotContainer().getDriveTrain().arcadeDrive(power, 0);
      }
    }
    else
    {
      if(holding)
      {
        Robot.getRobotContainer().getDriveTrain().moveStraight
          (encoderTargetToHold, HighAltitudeConstants.BALANCING_DEFAULT_POWER, angleTarget);
      }
      else
      {
        Robot.getRobotContainer().getDriveTrain().stop();
        encoderTargetToHold = Robot.getRobotContainer().getDriveTrain().getLeftEncoderDistance();
        holding = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }

  
  void checkAngles()
  {
    balanced = (Math.abs(Robot.getRobotContainer().getNavx().getPitch()) < 
      HighAltitudeConstants.BALANCING_ANGLE_THRESHOLD);

    stable = (Math.abs(Robot.getRobotContainer().getNavx().getAngularAccelerationPitch()) < 
      HighAltitudeConstants.BALANCING_ACCELERATION_THRESHOLD);
  }

}

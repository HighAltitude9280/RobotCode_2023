// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.primitives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.commands.autonomous.primitives.stepControl.MoveStraight;
import frc.robot.subsystems.chassis.DriveTrain;

public class AutoBalanceSimpleFwd extends CommandBase {
  DriveTrain driveTrain;
  double encoderTarget;
  boolean xd = false;

  /** Creates a new AutoBalanceAlreadyOnChargingStation. */
  public AutoBalanceSimpleFwd() {
    driveTrain = Robot.getRobotContainer().getDriveTrain();
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(0.125, 0);
    /*
     * if (Math.abs(Robot.getRobotContainer().getNavx().getYVel()) >
     * HighAltitudeConstants.BALANCING_SPEED_THRESHOLD) {
     * xd = true;
     * encoderTarget = driveTrain.getLeftEncoderDistance();
     * }
     * if (xd) {
     * driveTrain.moveStraight(encoderTarget, 0.5);
     * } else
     * driveTrain.arcadeDrive(0.1875, 0);
     */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // encoderTarget = driveTrain.getLeftEncoderDistance();
    // CommandScheduler.getInstance().schedule(new MoveStraight(encoderTarget,
    // 0.5));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return Math.abs(Robot.getRobotContainer().getNavx()
        .getYVel()) > HighAltitudeConstants.BALANCING_SPEED_THRESHOLD;
  }
}

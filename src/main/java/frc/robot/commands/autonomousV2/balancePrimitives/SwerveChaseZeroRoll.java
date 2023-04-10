// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2.balancePrimitives;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;

public class SwerveChaseZeroRoll extends CommandBase {
  SwerveDriveTrain swerveDriveTrain;
  boolean balanced;
  double currentRoll;

  /** Creates a new SwerveChaseZeroRoll. */
  public SwerveChaseZeroRoll() {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRoll = Robot.getRobotContainer().getNavx().getRoll();
    balanced = Math.abs(currentRoll) < 2.0;

    ChassisSpeeds chassisSpeeds;
    if (!balanced) {
      double error = 0 - currentRoll;
      double vx = error * 0.1;
      chassisSpeeds = new ChassisSpeeds(vx, 0.0, 0.0);
      SwerveModuleState[] moduleStates = HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      swerveDriveTrain.setModuleStates(moduleStates);
    } else {
      swerveDriveTrain.setModulesInXPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

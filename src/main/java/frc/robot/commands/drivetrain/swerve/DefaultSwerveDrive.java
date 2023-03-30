// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;

public class DefaultSwerveDrive extends CommandBase {
  private SlewRateLimiter speedLimiter, strafeLimiter, turnLimiter;
  SwerveDriveTrain swerveDriveTrain;

  /** Creates a new DriveSwerve. */
  public DefaultSwerveDrive() {
    speedLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    strafeLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND);
    turnLimiter = new SlewRateLimiter(HighAltitudeConstants.SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);

    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Read input
    double speed = OI.getInstance().getDefaultSwerveDriveSpeed();
    double strafe = OI.getInstance().getDefaultSwerveDriveStrafe();
    double turn = OI.getInstance().getDefaultSwerveDriveTurn();

    // 2. Limit the inputs' acceleration as to make driving smoother
    speed = speedLimiter.calculate(speed);
    strafe = strafeLimiter.calculate(strafe);
    turn = turnLimiter.calculate(turn);

    // 3. Scale input to teleop max speed
    speed *= HighAltitudeConstants.SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND;
    strafe *= HighAltitudeConstants.SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND;
    turn *= HighAltitudeConstants.SWERVE_DIRECTION_TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    // 4. Construct the chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (OI.getInstance().getSwerveDriveFieldOriented()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, strafe, turn,
          swerveDriveTrain.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(speed, strafe, turn);
    }

    // 5. Set the states to the swerve modules
    SwerveModuleState[] moduleStates = HighAltitudeConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerveDriveTrain.setModuleStates(moduleStates);

    // 6. Print for debugging
    SmartDashboard.putNumber("processed speed", speed);
    SmartDashboard.putNumber("processed strafe", strafe);
    SmartDashboard.putNumber("processed turn", turn);
    SmartDashboard.putNumber("FrontLeftSpeed", moduleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FrontLeftAngle", moduleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("FrontRightSpeed", moduleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("FrontRightAngle", moduleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("BackLeftSpeed", moduleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BackLeftAngle", moduleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("BackRightSpeed", moduleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("BackRightAngle", moduleStates[3].angle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

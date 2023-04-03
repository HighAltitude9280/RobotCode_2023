// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Extensor;
import frc.robot.subsystems.transport.Wrist;

public class ClearDangerousPositionBeforeTarget extends CommandBase {

  Wrist wrist;
  Arm arm;
  Extensor extensor;

  TransportTarget target;

  boolean wristArmDangerous, robotHeightDanger, armIntoFloorDanger, armIntoBackPlateDanger;

  double currentWristAngle, currentArmAngle, currentExtensorDistance;
  double wristTarget, armTarget, extensorTarget;

  GamePieceMode initialGamePieceMode;

  boolean hasReachedArmWristTarget = false;

  /** Creates a new ClearDangerousPosition. */
  public ClearDangerousPositionBeforeTarget(TransportTarget target) {
    wrist = Robot.getRobotContainer().getWrist();
    arm = Robot.getRobotContainer().getArm();
    extensor = Robot.getRobotContainer().getExtensor();

    this.target = target;
    addRequirements(wrist, arm, extensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initialGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();

    switch (initialGamePieceMode) {
      case CONE:
        wristTarget = target.getWristTargetCone();
        extensorTarget = target.getExtensorTargetCone();
        armTarget = target.getArmTargetCone();
        break;
      case CUBE:
        wristTarget = target.getWristTargetCube();
        extensorTarget = target.getExtensorTargetCube();
        armTarget = target.getArmTargetCube();
        break;
      case MANUAL:
        super.cancel();
        break;
      default:
        super.cancel();
        break;
    }

    currentWristAngle = wrist.getCurrentAngle();
    currentArmAngle = arm.getCurrentAngle();
    currentExtensorDistance = extensor.getCurrentDistance();
    double armWristDelta = currentArmAngle - currentWristAngle;

    wristArmDangerous = (armWristDelta < -41.0 || armWristDelta > 110.0)
        && Math.abs(currentWristAngle - wristTarget) > 18.0;

    double maxArmAngle = Math.max(currentArmAngle, armTarget);
    double minArmAngle = Math.min(currentArmAngle, armTarget);

    boolean goesVertical = (maxArmAngle > 5 && minArmAngle < 5) ||
        (maxArmAngle > -35 && minArmAngle < -35);

    robotHeightDanger = (currentExtensorDistance > 0.05) && goesVertical;

    armIntoFloorDanger = (robotHeightDanger || currentExtensorDistance < 0.25)
        && (currentArmAngle < -120.0 || armTarget < -120.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armIntoFloorDanger) {
      boolean armSafe = arm.moveTo(-110, HighAltitudeConstants.ARM_AUTO_MAX_POWER);
      boolean wristSafe = wrist.moveTo(-170, HighAltitudeConstants.WRIST_AUTO_MAX_POWER);
      boolean extensorTargetStillNeedsToGoUp = extensorTarget > extensor.getCurrentDistance();
      if (armSafe && wristSafe) {
        if (extensorTargetStillNeedsToGoUp) {
          if (extensor.moveTo(extensorTarget, HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER)) {
            armIntoFloorDanger = false;
            wristArmDangerous = false;
          }
        } else {
          armIntoFloorDanger = false;
          wristArmDangerous = false;
        }
      }

    } else if (wristArmDangerous) {
      if (wrist.moveTo(currentArmAngle - 50, HighAltitudeConstants.WRIST_AUTO_MAX_POWER)) {
        wristArmDangerous = false;
      }
    }
    if (robotHeightDanger) {
      boolean isTargetAlreadyLower = extensorTarget < extensor.getCurrentDistance();
      double targetIfHeightDanger = isTargetAlreadyLower ? extensorTarget : 0.0;
      if (extensor.moveTo(targetIfHeightDanger, HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER)) {
        robotHeightDanger = false;
      }
    }
    if (!robotHeightDanger && !wristArmDangerous && !armIntoFloorDanger) {
      boolean hasReachedArmTarget = arm.moveTo(armTarget, target.getArmMaxPower());
      boolean hasReachedWristTarget = wrist.moveTo(wristTarget, target.getWristMaxPower());
      hasReachedArmWristTarget = hasReachedArmTarget && hasReachedWristTarget;
    }

    double maxArmAngle = Math.max(arm.getCurrentAngle(), armTarget);
    double minArmAngle = Math.min(arm.getCurrentAngle(), armTarget);

    boolean goesVertical = (maxArmAngle > 5 && minArmAngle < 5) ||
        (maxArmAngle > -35 && minArmAngle < -35);

    if (!goesVertical && !robotHeightDanger && !hasReachedArmWristTarget && !armIntoFloorDanger) {
      extensor.moveTo(extensorTarget, target.getExtensorMaxPower());
    }

    SmartDashboard.putBoolean("GoesVertical", goesVertical);
    SmartDashboard.putBoolean("robotHeightDanger", robotHeightDanger);
    SmartDashboard.putBoolean("hasReachedArmWristTarget", hasReachedArmWristTarget);
    SmartDashboard.putBoolean("wristarmdangerous", wristArmDangerous);
    SmartDashboard.putBoolean("armIntoFloorDanger", armIntoFloorDanger);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.getRobotContainer().getCurrentGamePieceMode().equals(GamePieceMode.MANUAL))
      return true;
    // System.out.println("HasFinished. reachedarwrist: " +
    // hasReachedArmWristTarget);
    return wristArmDangerous == false && robotHeightDanger == false && hasReachedArmWristTarget == true
        && armIntoFloorDanger == false;
  }
}

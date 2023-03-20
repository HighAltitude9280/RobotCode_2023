// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

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

  boolean wristArmDangerous, robotHeightDanger;

  double currentWristAngle, currentArmAngle, currentExtensorDistance;
  double wristTarget, armTarget, extensorTarget;

  GamePieceMode initialGamePieceMode;

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

    wristArmDangerous = (armWristDelta < -139 || armWristDelta > 47)
        && Math.abs(currentWristAngle - wristTarget) > 8.0;

    double maxArmAngle = Math.max(currentArmAngle, armTarget);
    double minArmAngle = Math.min(currentArmAngle, armTarget);

    boolean goesVertical = (maxArmAngle > 110 && minArmAngle < 110) ||
        (maxArmAngle > 80 && minArmAngle < 80);

    robotHeightDanger = (currentExtensorDistance > 0.1) && goesVertical;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wristArmDangerous) {
      if (wrist.moveTo(currentArmAngle + 70, HighAltitudeConstants.WRIST_AUTO_MAX_POWER)) {
        wristArmDangerous = false;
      }
    }
    if (robotHeightDanger) {
      boolean isTargetAlreadyLower = extensorTarget < extensor.getCurrentDistance();
      double targetIfHeightDanger = isTargetAlreadyLower ? extensorTarget : 0.1;
      if (extensor.moveTo(targetIfHeightDanger, HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER)) {
        robotHeightDanger = false;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristArmDangerous == false && robotHeightDanger == false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.resources.math.Math;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Extensor;
import frc.robot.subsystems.transport.Wrist;

public class NewTransportGoTo extends CommandBase {
  Wrist wrist;
  Arm arm;
  Extensor extensor;

  TransportTarget target;
  GamePieceMode currentGamePieceMode;

  double wristFinalTarget, armFinalTarget, extensorFinalTarget;
  double wristCurrentTarget, armCurrentTarget, extensorCurrentTarget;

  PIDController wristPIDController, armPIDController, extensorPIDController;

  boolean wristArmDangerous, armIntoBackPlateDangerGoingDown, armIntoBackPlateDangerGoingUp, armIntoFloorDangerGoingUp;

  final double ARM_WRIST_DELTA_LOWER_LIMIT = HighAltitudeConstants.WRIST_ARM_DELTA_LOWER_LIMIT;
  final double ARM_WRIST_DELTA_UPPER_LIMIT = HighAltitudeConstants.WRIST_ARM_DELTA_UPPER_LIMIT;
  final double EXTENSOR_UP_AND_ARM_MIGHT_CRASH_WITH_BACKPLATE = 0.43; // maybe
  final double ARM_BACKPLATE_SAFE_POSITION = 29.0; // maybe
  final double WRIST_BACKPLATE_SAFE_POSITION = -2.5; // maybe
  final double ARM_HORIZONTAL = -85.0; // maybe
  final double EXTENSOR_DOWN_AND_ARM_MIGHT_CRASH_WITH_FLOOR = 0.2; // maybe

  /** Creates a new NewTransportGoTo. */
  public NewTransportGoTo(TransportTarget target) {
    wrist = Robot.getRobotContainer().getWrist();
    arm = Robot.getRobotContainer().getArm();
    extensor = Robot.getRobotContainer().getExtensor();

    addRequirements(wrist, arm, extensor);

    wristPIDController = new PIDController(1 / (HighAltitudeConstants.WRIST_AUTO_MAX_POWER
        * HighAltitudeConstants.WRIST_AUTO_MAX_POWER * HighAltitudeConstants.WRIST_BRAKING_DEGREES), 0, 0.05);

    armPIDController = new PIDController(1 / (HighAltitudeConstants.ARM_AUTO_MAX_POWER
        * HighAltitudeConstants.ARM_AUTO_MAX_POWER * HighAltitudeConstants.ARM_BRAKING_DEGREES), 0, 0.1);

    extensorPIDController = new PIDController(1 / (HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER
        * HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER * HighAltitudeConstants.EXTENSOR_BRAKING_METERS), 0, 0.025);
  }

  public NewTransportGoTo(TransportTarget target, PIDConstants wristPIDConstants, PIDConstants armPIDConstants,
      PIDConstants extensorPIDConstants) {

    wrist = Robot.getRobotContainer().getWrist();
    arm = Robot.getRobotContainer().getArm();
    extensor = Robot.getRobotContainer().getExtensor();

    addRequirements(wrist, arm, extensor);

    wristPIDController = new PIDController(wristPIDConstants.kP, wristPIDConstants.kI, wristPIDConstants.kD);
    armPIDController = new PIDController(armPIDConstants.kP, armPIDConstants.kI, armPIDConstants.kD);
    wristPIDController = new PIDController(extensorPIDConstants.kP, extensorPIDConstants.kI, extensorPIDConstants.kD);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();

    switch (currentGamePieceMode) {
      case CONE:
        wristFinalTarget = target.getWristTargetCone();
        extensorFinalTarget = target.getExtensorTargetCone();
        armFinalTarget = target.getArmTargetCone();
        break;
      case CUBE:
        wristFinalTarget = target.getWristTargetCube();
        extensorFinalTarget = target.getExtensorTargetCube();
        armFinalTarget = target.getArmTargetCube();
        break;
      default:
      case MANUAL:
        wristFinalTarget = wrist.getCurrentAngle();
        armFinalTarget = arm.getCurrentAngle();
        extensorFinalTarget = extensor.getCurrentDistance();
        super.cancel();
        break;
    }

    // Check dangers
    double wristCurrentAngle = wrist.getCurrentAngle();
    double armCurrentAngle = arm.getCurrentAngle();
    double extensorCurrentDistance = extensor.getCurrentDistance();

    // Check if wrist and arm are in dangerous positions

    // True if wrist is outside its safe zone and if target doesn't change much from
    // current angle.
    double armWristDelta = armCurrentAngle - wristCurrentAngle;
    boolean wristIsVeryCloseToTargetAlready = Math.abs(wristFinalTarget - wristCurrentAngle) < 0.0;

    wristArmDangerous = !isWithinRange(armWristDelta, ARM_WRIST_DELTA_LOWER_LIMIT, ARM_WRIST_DELTA_UPPER_LIMIT)
        && !wristIsVeryCloseToTargetAlready;

    // Check if arm might crash into backplate

    // 1st case: extensor needs to go down and arm exceeds safe angle
    boolean extensorIsUpAndNeedsToGoDown = extensorCurrentDistance > EXTENSOR_UP_AND_ARM_MIGHT_CRASH_WITH_BACKPLATE
        && extensorFinalTarget < EXTENSOR_UP_AND_ARM_MIGHT_CRASH_WITH_BACKPLATE;
    boolean armIsNotInSafeZone = armCurrentAngle > ARM_BACKPLATE_SAFE_POSITION;
    armIntoBackPlateDangerGoingDown = extensorIsUpAndNeedsToGoDown && armIsNotInSafeZone;

    // 2nd case: extensor needs to go up and arm needs to exceed safe angle
    boolean extensorIsDownAndNeedsToGoUp = extensorCurrentDistance < 0.5 && extensorFinalTarget > 0.5;
    boolean armNeedsToExceedSafeAngle = armFinalTarget > ARM_BACKPLATE_SAFE_POSITION;
    armIntoBackPlateDangerGoingUp = extensorIsDownAndNeedsToGoUp && armNeedsToExceedSafeAngle;

    // Check if arm might crash into floor;
    boolean armHorizontalAndNeedsToGoDown = armCurrentAngle < ARM_HORIZONTAL && armFinalTarget < armCurrentAngle;
    boolean extensorIsFullyDownAndNeedsToGoUp = extensorCurrentDistance < EXTENSOR_DOWN_AND_ARM_MIGHT_CRASH_WITH_FLOOR
        && extensorFinalTarget > extensorCurrentDistance;
    armIntoFloorDangerGoingUp = armHorizontalAndNeedsToGoDown && extensorIsFullyDownAndNeedsToGoUp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wristCurrentAngle = wrist.getCurrentAngle();
    double armCurrentAngle = arm.getCurrentAngle();
    double extensorCurrentDistance = extensor.getCurrentDistance();

    wristCurrentTarget = wristFinalTarget;
    armCurrentTarget = armFinalTarget;
    extensorCurrentTarget = extensorFinalTarget;

    // If there's a danger that the arm might crash into the floor while moving up,
    // do not move the arm until the extensor elevates it to a safe position.
    if (armIntoFloorDangerGoingUp) {
      armCurrentTarget = armCurrentAngle;
      if (extensorCurrentDistance > 0.1) {
        armIntoFloorDangerGoingUp = false;
        armCurrentTarget = armFinalTarget;
      }

    }

    // If the wrist-arm delta is dangerous DO NOT move the arm until the wrist is in
    // a safe position.
    if (wristArmDangerous) {
      armCurrentTarget = armCurrentAngle;
      wristCurrentTarget = armCurrentAngle + 0.0;
      if (isInAcceptedRange(wristCurrentAngle, wristCurrentTarget,
          HighAltitudeConstants.WRIST_ARRIVE_OFFSET)) {
        wristArmDangerous = false;

        armCurrentTarget = armFinalTarget;
        wristCurrentTarget = wristFinalTarget;
      }
    }

    // If there is no wrist-arm danger and there is a danger that the arm might
    // crash into the backplate while going up, DO NOT move the wrist-arm past the
    // safe position until extensor elevates them to a safe position.
    if (armIntoBackPlateDangerGoingUp && !wristArmDangerous) {
      armCurrentTarget = 0.0;
      wristCurrentTarget = 0.0;

      boolean armHasReachedSafeZone = isInAcceptedRange(armCurrentAngle, armCurrentTarget,
          HighAltitudeConstants.ARM_ARRIVE_OFFSET);
      boolean wristHasReachedSafeZone = isInAcceptedRange(wristCurrentAngle, wristCurrentTarget,
          HighAltitudeConstants.WRIST_ARRIVE_OFFSET);

      if (extensorCurrentDistance > 0.5 && armHasReachedSafeZone && wristHasReachedSafeZone) {
        armIntoBackPlateDangerGoingUp = false;
        armCurrentTarget = armFinalTarget;
        wristCurrentAngle = wristFinalTarget;
      }

    }

    // If there is no wrist-arm danger and there is a danger that the arm might
    // crash into the backplate while going down, DO NOT move the extensor below the
    // safe position until the wrist-arm position crosses to a safe position.
    if (armIntoBackPlateDangerGoingDown && !wristArmDangerous) {
      extensorCurrentTarget = extensorCurrentDistance;

      boolean armHasReachedSafeZone = armCurrentAngle < 0.0;
      boolean wristHasReachedSafeZone = wristCurrentAngle < 0.0;

      if (armHasReachedSafeZone && wristHasReachedSafeZone) {
        armIntoBackPlateDangerGoingDown = false;
        extensorCurrentTarget = extensorFinalTarget;
      }
    }

    SmartDashboard.putBoolean("wristarmdelta", wristArmDangerous);
    SmartDashboard.putBoolean("arm to backplate going up", armIntoBackPlateDangerGoingUp);
    SmartDashboard.putBoolean("arm to backplate going down", armIntoBackPlateDangerGoingDown);
    SmartDashboard.putBoolean("armIntoFloorGoingUp", armIntoFloorDangerGoingUp);
    setPIDOutputs(wristCurrentTarget, armCurrentTarget, extensorCurrentTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.driveWrist(0);
    extensor.driveExtensor(0);
    arm.driveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentGamePieceMode.equals(GamePieceMode.MANUAL))
      return true;

    boolean wristInTarget = isInAcceptedRange(wrist.getCurrentAngle(), wristFinalTarget,
        HighAltitudeConstants.WRIST_ARRIVE_OFFSET);
    boolean armInTarget = isInAcceptedRange(arm.getCurrentAngle(), armFinalTarget,
        HighAltitudeConstants.ARM_ARRIVE_OFFSET);
    boolean extensorInTarget = isInAcceptedRange(extensor.getCurrentDistance(), extensorFinalTarget,
        HighAltitudeConstants.EXTENSOR_ARRIVE_OFFSET);

    return wristInTarget == true && armInTarget == true && extensorInTarget == true;
  }

  void setPIDOutputs(double wristTarget, double armTarget, double extensorTarget) {
    double wristOutput = wristPIDController.calculate(wrist.getCurrentAngle(), wristTarget);
    wristOutput = Math.clamp(wristOutput, -1.0, 1.0) * HighAltitudeConstants.WRIST_AUTO_MAX_POWER;
    wrist.driveWrist(wristOutput);

    double armOutput = armPIDController.calculate(arm.getCurrentAngle(), armTarget);
    armOutput = Math.clamp(armOutput, -1.0, 1.0) * HighAltitudeConstants.ARM_AUTO_MAX_POWER;
    arm.driveArm(armOutput);

    double extensorOutput = extensorPIDController.calculate(extensor.getCurrentDistance(), extensorTarget);
    extensorOutput = Math.clamp(extensorOutput, -1.0, 1.0) * HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER;
    extensor.driveExtensor(extensorOutput);
  }

  boolean isInAcceptedRange(double currentPosition, double targetPosition, double acceptedRange) {
    return Math.abs(targetPosition - currentPosition) < acceptedRange;
  }

  boolean isWithinRange(double x, double mn, double mx) {
    return x > mn && x < mx;
  }
}

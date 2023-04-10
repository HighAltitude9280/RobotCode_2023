// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Wrist;

public class SimultaneousArmWristMovement3 extends CommandBase {
  Arm arm;
  Wrist wrist;
  double delta;

  /** Creates a new SimultaneousArmWristMovement3. */
  public SimultaneousArmWristMovement3() {
    arm = Robot.getRobotContainer().getArm();
    wrist = Robot.getRobotContainer().getWrist();
    addRequirements(arm, wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delta = arm.getCurrentAngle() - wrist.getCurrentAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = OI.getInstance().getArmInput();
    double wristSpeed = OI.getInstance().getWristInput();

    arm.driveArm(armSpeed);
    // If the wrist IS receiving an input, drive it to
    // that input and update the delta
    if (wristSpeed != 0) {
      delta = arm.getCurrentAngle() - wrist.getCurrentAngle();
      if ((Robot.getRobotContainer().getShouldManualHaveLimits())
          && ((delta < HighAltitudeConstants.WRIST_ARM_DELTA_LOWER_LIMIT && wristSpeed > 0)
              || (delta > HighAltitudeConstants.WRIST_ARM_DELTA_UPPER_LIMIT && wristSpeed < 0)))
        wrist.driveWrist(0);
      else
        wrist.driveWrist(wristSpeed);
    }
    // If it's not in manual mode (if it's in game piece mode), compensate the angle
    else if (Robot.getRobotContainer().getCurrentGamePieceMode() != GamePieceMode.MANUAL) {
      wrist.moveTo(arm.getCurrentAngle() - delta, Math.abs(armSpeed * 0.575));
    }
    // If it's not being driven and it's in manual, just stop it.
    else
      wrist.driveWrist(0);

    Robot.debugNumberSmartDashboard("Wrist Current Ang", wrist.getCurrentAngle());
    Robot.debugNumberSmartDashboard("Wrist Target", (arm.getCurrentAngle() - delta));
    Robot.debugNumberSmartDashboard("Arm Current Ang", arm.getCurrentAngle());
    Robot.debugNumberSmartDashboard("Delta", delta);
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

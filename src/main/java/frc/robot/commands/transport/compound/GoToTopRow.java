// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.transport.arm.DriveArmToPosition;
import frc.robot.commands.transport.extensor.DriveExtensorToPosition;
import frc.robot.commands.transport.wrist.DriveWristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToTopRow extends SequentialCommandGroup {
  GamePieceMode currentGamePieceMode;
  double extensorTarget, armTarget, wristTarget;
  double extensorMaxPower, armMaxPower, wristMaxPower;

  /** Creates a new GoToTopRow. */
  public GoToTopRow() {
    currentGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();

    extensorMaxPower = 0.3;
    armMaxPower = 0.3;
    wristMaxPower = 0.3;

    switch (currentGamePieceMode) {
      case CONE:
        extensorTarget = HighAltitudeConstants.EXTENSOR_TOP_ROW_METERS_CONE;
        armTarget = HighAltitudeConstants.ARM_TOP_ROW_DEGREES_CONE;
        wristTarget = HighAltitudeConstants.WRIST_TOP_ROW_DEGREES_CONE;
        break;
      case CUBE:
        extensorTarget = HighAltitudeConstants.EXTENSOR_TOP_ROW_METERS_CUBE;
        armTarget = HighAltitudeConstants.ARM_TOP_ROW_DEGREES_CUBE;
        wristTarget = HighAltitudeConstants.WRIST_TOP_ROW_DEGREES_CUBE;
        break;
      case OTHER:
        return;
      default:
        return;
    }

    addCommands(
        new DriveExtensorToPosition(extensorTarget, extensorMaxPower),
        new DriveArmToPosition(armTarget, armMaxPower),
        new DriveWristToPosition(wristTarget, wristMaxPower));
  }
}

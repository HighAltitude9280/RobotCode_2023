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
public class GoToIntake extends SequentialCommandGroup {
  GamePieceMode currentGamePieceMode;
  double extensorTarget, armTarget, wristTarget;
  double extensorMaxPower, armMaxPower, wristMaxPower;

  /** Creates a new GoToTopRow. */
  public GoToIntake() {
    currentGamePieceMode = Robot.getRobotContainer().getCurrentGamePieceMode();

    wristMaxPower = HighAltitudeConstants.WRIST_AUTO_MAX_POWER;
    armMaxPower = HighAltitudeConstants.ARM_AUTO_MAX_POWER;
    extensorMaxPower = HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER;

    switch (currentGamePieceMode) {
      case CONE:
        wristTarget = HighAltitudeConstants.WRIST_INTAKE_DEGREES_CONE;
        armTarget = HighAltitudeConstants.ARM_INTAKE_DEGREES_CONE;
        extensorTarget = HighAltitudeConstants.EXTENSOR_INTAKE_METERS_CONE;
        break;
      case CUBE:
        wristTarget = HighAltitudeConstants.WRIST_INTAKE_DEGREES_CUBE;
        armTarget = HighAltitudeConstants.ARM_INTAKE_DEGREES_CUBE;
        extensorTarget = HighAltitudeConstants.EXTENSOR_INTAKE_METERS_CUBE;
        break;
      case MANUAL:
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

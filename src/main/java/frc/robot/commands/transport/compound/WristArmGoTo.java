// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotContainer.GamePieceMode;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.arm.DriveArmToTarget;
import frc.robot.commands.transport.wrist.DriveWristToTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristArmGoTo extends SequentialCommandGroup {
  GamePieceMode currentGamePieceMode;
  double extensorTarget, armTarget, wristTarget;
  double extensorMaxPower, armMaxPower, wristMaxPower;

  TransportTarget target;

  /** Creates a new WristArmGoTo. */
  public WristArmGoTo(TransportTarget target) {
    wristMaxPower = HighAltitudeConstants.WRIST_AUTO_MAX_POWER;
    armMaxPower = HighAltitudeConstants.ARM_AUTO_MAX_POWER;
    extensorMaxPower = HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER;

    addCommands(
        Commands.parallel(
            new DriveArmToTarget(target, target.getArmMaxPower()),
            new DriveWristToTarget(target, target.getWristMaxPower())));
  }
}

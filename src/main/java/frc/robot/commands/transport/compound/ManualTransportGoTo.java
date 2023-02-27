// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transport.compound;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.transport.arm.DriveArmToPosition;
import frc.robot.commands.transport.extensor.DriveExtensorToPosition;
import frc.robot.commands.transport.wrist.DriveWristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualTransportGoTo extends SequentialCommandGroup {
  /** Creates a new ManualTransportGoTo. */
  public ManualTransportGoTo(double armTarget, double armMaxPower, double extensorTarget, double extensorMaxPower,
      double wristTarget, double wristMaxPower) {
    addCommands(
        Commands.parallel(
            new DriveArmToPosition(armTarget, armMaxPower),
            new DriveExtensorToPosition(extensorTarget, extensorMaxPower)),
        new DriveWristToPosition(wristTarget, wristMaxPower));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.primitives.transport;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.transport.arm.DriveArmToPosition;
import frc.robot.commands.transport.extensor.DriveExtensorToPosition;
import frc.robot.commands.transport.wrist.DriveWristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BreakInitialConfig extends SequentialCommandGroup {
  /** Creates a new BreakInitialConfig. */
  public BreakInitialConfig() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // TODO: check if this is appropiate
    addCommands(new DriveExtensorToPosition(0.2, HighAltitudeConstants.EXTENSOR_AUTO_MAX_POWER),
        Commands.parallel(
            new DriveArmToPosition(20, HighAltitudeConstants.ARM_AUTO_MAX_POWER),
            new DriveWristToPosition(40, HighAltitudeConstants.WRIST_AUTO_MAX_POWER)));
  }
}

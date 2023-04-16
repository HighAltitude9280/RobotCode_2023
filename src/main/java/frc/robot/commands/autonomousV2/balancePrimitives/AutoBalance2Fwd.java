// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousV2.balancePrimitives;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.swerve.SwerveDriveDistanceFwd;
import frc.robot.commands.drivetrain.swerve.SwerveSetX;
import frc.robot.commands.drivetrain.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.resources.components.PWMLEDStrip.commands.primitives.SetRGBDontTurnOff;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance2Fwd extends SequentialCommandGroup {
  /** Creates a new AutoBalance2Fwd. */
  public AutoBalance2Fwd() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*
     * // ESTE FUNCIONÓ DE ALGUNA FORMA U OTRA PERO MUY LENTO
     * addCommands(
     * new ResetOdometryZeros(),
     * new SwerveFwdUntilAngleChange(1.0, 8.0),
     * new SwerveDriveDistanceFwd(0.9, 0.85, true).withTimeout(4.5),
     * new SwerveWaitForFall(0.5, 12.5));
     */
    // ESTE FUNCIONÓ DE ALGUNA FORMA U OTRA PERO MUY LENTO
    addCommands(
        new ResetOdometryZeros(),
        new SwerveFwdUntilAngleChange(1.0, 8.0),
        new ScheduleCommand(new SetRGBDontTurnOff(0, 255, 0).withTimeout(1.0)),
        new SwerveDriveDistanceFwd(0.9, 0.85, true).withTimeout(2.0),
        new ScheduleCommand(new SetRGBDontTurnOff(0, 255, 0).withTimeout(1.0)),
        new SwerveWaitForFall(0.6, 7.5),
        new ScheduleCommand(new SetRGBDontTurnOff(0, 0, 255).withTimeout(1.0)),
        new SwerveSetX());

    /*
     * EESTE NO XD
     * addCommands(
     * new SwerveFwdUntilAngleChange(1.25, 8.0),
     * new SwerveDriveDistanceFwd(1.0, 0.85, true).withTimeout(4.5),
     * new SwerveWaitForFall(0.8, 12.5));
     */
  }
}

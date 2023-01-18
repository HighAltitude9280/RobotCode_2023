// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DefaultDrive;
import frc.robot.resources.components.Navx;
<<<<<<< Updated upstream
import frc.robot.resources.components.PWMLEDStrip.LEDs;
import frc.robot.subsystems.chassis.DriveTrain;
=======
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
>>>>>>> Stashed changes

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  private Command m_autoCommand;

  private Navx navx;
  private DriveTrain driveTrain;
<<<<<<< Updated upstream
  private LEDs leds;
=======
  private Vision parkerVision;
>>>>>>> Stashed changes

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    navx = new Navx();
    leds = new LEDs();
    driveTrain = new DriveTrain();

    parkerVision = new Vision();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
<<<<<<< Updated upstream
  public void configureRobotContainer() {
=======
  public void configureButtonBindings() {
>>>>>>> Stashed changes
    driveTrain.setDefaultCommand(new DefaultDrive());

    OI.getInstance().ConfigureButtonBindings();
    leds.allLedsOff();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Navx getNavx() {
    return navx;
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

  public Vision getVision() {
    return parkerVision;
  }
}

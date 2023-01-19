// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DefaultDrive;
import frc.robot.commands.transport.arm.DriveArm;
import frc.robot.commands.transport.wrist.DriveWrist;
import frc.robot.resources.components.Navx;
import frc.robot.resources.components.PWMLEDStrip.LEDs;
import frc.robot.resources.components.PWMLEDStrip.commands.DisplayGamePieceMode;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Extensor;
import frc.robot.subsystems.transport.Wrist;
import frc.robot.subsystems.Vision;

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

  public enum GamePieceMode {
    CUBE, CONE, OTHER
  }

  private Command m_autoCommand;

  private Navx navx;
  private DriveTrain driveTrain;
  private LEDs leds;
  private Vision parkerVision;
  private Wrist wrist;
  private Gripper gripper;
  private Arm arm;
  private Extensor extensor;

  private GamePieceMode currentGamePieceMode;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    navx = new Navx();
    leds = new LEDs();
    driveTrain = new DriveTrain();
    wrist = new Wrist();
    gripper = new Gripper();
    parkerVision = new Vision();
    arm = new Arm();

    currentGamePieceMode = GamePieceMode.OTHER;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureRobotContainer() {
    driveTrain.setDefaultCommand(new DefaultDrive());
    wrist.setDefaultCommand(new DriveWrist()); // POV-Y
    arm.setDefaultCommand(new DriveArm()); // Triggers

    leds.setDefaultCommand(new DisplayGamePieceMode());

    OI.getInstance().ConfigureButtonBindings();
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

  public Wrist getWrist() {
    return wrist;
  }

  public Gripper getGripper() {
    return gripper;
  }

  public LEDs getLeds() {
    return leds;
  }

  public Vision getVision() {
    return parkerVision;
  }

  public Arm getArm() {
    return arm;
  }

  public Extensor getExtensor() {
    return extensor;
  }

  public void setCurrentGamePieceMode(GamePieceMode mode) {
    currentGamePieceMode = mode;
  }

  public GamePieceMode getCurrentGamePieceMode() {
    return currentGamePieceMode;
  }

}

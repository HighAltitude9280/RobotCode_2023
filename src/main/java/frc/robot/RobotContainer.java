// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.Paths;
import frc.robot.commands.autonomous.primitives.stepControl.SplineMove;
import frc.robot.commands.drivetrain.DefaultDrive;
import frc.robot.commands.transport.arm.DriveArm;
import frc.robot.commands.transport.extensor.DriveExtensor;
import frc.robot.commands.transport.wrist.DriveWrist;
import frc.robot.resources.components.Navx;
import frc.robot.resources.components.PWMLEDStrip.LEDs;
import frc.robot.resources.components.PWMLEDStrip.commands.DisplayGamePieceMode;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transport.Arm;
import frc.robot.subsystems.transport.Extensor;
import frc.robot.subsystems.transport.Wrist;
import frc.robot.subsystems.vision.DriverCameras;
import frc.robot.subsystems.vision.LimeLightVision;

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
    CUBE, CONE, MANUAL
  }

  private Command m_autoCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private GamePieceMode currentGamePieceMode;
  private Navx navx;
  private DriveTrain driveTrain;
  private Wrist wrist;
  private Arm arm;
  private Extensor extensor;
  private Gripper gripper;
  private Intake intake;
  private LimeLightVision limeLightVision;
  private DriverCameras driverCameras;
  private LEDs leds;
  // IMPORTANT: with this boolean in false, limits won't affect manual movement
  private boolean shouldManualBeLimited = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    navx = new Navx();
    driveTrain = new DriveTrain();
    wrist = new Wrist();
    arm = new Arm();
    extensor = new Extensor();
    gripper = new Gripper();
    intake = new Intake();

    limeLightVision = new LimeLightVision();
    driverCameras = new DriverCameras();
    leds = new LEDs();
    currentGamePieceMode = GamePieceMode.CONE;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    driveTrain.setDefaultCommand(new DefaultDrive());
    wrist.setDefaultCommand(new DriveWrist());
    arm.setDefaultCommand(new DriveArm());
    extensor.setDefaultCommand(new DriveExtensor());
    leds.setDefaultCommand(new DisplayGamePieceMode());
    OI.getInstance().ConfigureButtonBindings();
  }

  public void generateAutos() {
    SplineMove followExamplePath = new SplineMove(Paths.examplePath, 0.5,
        true, false, false, false);

    m_chooser.setDefaultOption("Example path", followExamplePath);

    m_autoCommand = followExamplePath;
  }

  public void putAutoChooser() {
    SmartDashboard.putData("Autonomous", m_chooser);
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

  public Arm getArm() {
    return arm;
  }

  public Extensor getExtensor() {
    return extensor;
  }

  public Intake getIntake() {
    return intake;
  }

  public LEDs getLeds() {
    return leds;
  }

  public LimeLightVision getLimeLightVision() {
    return limeLightVision;
  }

  public DriverCameras getDriverCameras() {
    return driverCameras;
  }

  public void setCurrentGamePieceMode(GamePieceMode mode) {
    currentGamePieceMode = mode;
  }

  public GamePieceMode getCurrentGamePieceMode() {
    return currentGamePieceMode;
  }

  public void setShouldManualBeLimited(boolean shouldBeLimited) {
    shouldManualBeLimited = shouldBeLimited;
  }

  public boolean getShouldManualBeLimited() {
    return shouldManualBeLimited;
  }
}

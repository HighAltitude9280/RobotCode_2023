// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.sequences.fullAutonomous.standard.DoNothing;
import frc.robot.commands.autonomousV2.SwerveAutos;
import frc.robot.commands.autonomousV2.SwerveAutos2;
import frc.robot.commands.autonomousV2.balancePrimitives.AutoBalance2Bwd;
import frc.robot.commands.autonomousV2.balancePrimitives.AutoBalance2Fwd;
import frc.robot.commands.autonomousV2.superSimpleAutos.LeavePiece;
import frc.robot.commands.autonomousV2.superSimpleAutos.LeavePieceThenRest;
import frc.robot.commands.drivetrain.swerve.DefaultSwerveDrive;
import frc.robot.commands.pieceHandlers.gripper.DriveGripper;
import frc.robot.commands.transport.TransportTargets.TransportTarget;
import frc.robot.commands.transport.compound.NewTransportGoTo;
import frc.robot.commands.transport.compound.SimultaneousArmWristMovement3;
import frc.robot.commands.transport.extensor.DriveExtensor;
import frc.robot.resources.components.Navx;
import frc.robot.resources.components.PWMLEDStrip.LEDs;
import frc.robot.resources.components.PWMLEDStrip.commands.DisplayGamePieceModeVariableIntensity;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.chassis.swerve.SwerveDriveTrain;
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

  // private Command m_autoCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private GamePieceMode currentGamePieceMode = GamePieceMode.MANUAL;
  private Navx navx;
  private DriveTrain driveTrain;
  private SwerveDriveTrain swerveDriveTrain;
  private Wrist wrist;
  private Arm arm;
  private Extensor extensor;
  private Gripper gripper;
  private Intake intake;
  private LimeLightVision limeLightVision;
  private DriverCameras driverCameras;
  private LEDs leds;
  // IMPORTANT: with this boolean in false, limits won't affect manual movement
  private boolean shouldManualHaveLimits = true;
  private boolean shouldExtensorBeSlowerInManual = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    navx = new Navx();
    // driveTrain = new DriveTrain();
    swerveDriveTrain = new SwerveDriveTrain();
    wrist = new Wrist();
    arm = new Arm();
    extensor = new Extensor();
    gripper = new Gripper();
    // intake = new Intake();

    limeLightVision = new LimeLightVision();
    // driverCameras = new DriverCameras();
    leds = new LEDs();
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
    // driveTrain.setDefaultCommand(new DefaultDrive());
    wrist.setDefaultCommand(new SimultaneousArmWristMovement3());
    arm.setDefaultCommand(new SimultaneousArmWristMovement3());
    extensor.setDefaultCommand(new DriveExtensor());
    leds.setDefaultCommand(new DisplayGamePieceModeVariableIntensity());
    OI.getInstance().ConfigureButtonBindings();

    gripper.setDefaultCommand(new DriveGripper());

    // swerveDriveTrain.setDefaultCommand(new DefaultSwerveDrive());
    swerveDriveTrain.setDefaultCommand(new DefaultSwerveDrive());
  }

  public void generateAutos() {
    // SplineMove followExamplePath = new SplineMove(Paths.examplePath, 0.1,
    // true, false, false, false);
    // BreakInitialConfig breakInitialConfig = new BreakInitialConfig();
    // PreloadedPieceThenCharging coneThenCharging = new
    // PreloadedPieceThenCharging(GamePieceMode.CONE);
    // PreloadedPieceThenCharging cubeThenCharging = new&
    // PreloadedPieceThenCharging(GamePieceMode.CUBE);
    // Charging charging = new Charging();
    // Forward forward = new Forward();
    // PreloadedPieceOnly preloadedConeOnly = new
    // PreloadedPieceOnly(GamePieceMode.CONE);
    // PreloadedPieceOnly preloadedCubeOnly = new
    // PreloadedPieceOnly(GamePieceMode.CUBE);

    // FasterPreloadedPieceThenMoveStraight coneThenMove = new
    // FasterPreloadedPieceThenMoveStraight(GamePieceMode.CONE);
    // FasterPreloadedPieceThenMoveStraight cubeThenMove = new
    // FasterPreloadedPieceThenMoveStraight(GamePieceMode.CUBE);
    // FasterPreloadedPieceOnly coneThenStayStill = new FasterPreloadedPieceOnly(
    // GamePieceMode.CONE);
    // FasterPreloadedPieceOnly cubeThenStayStill = new FasterPreloadedPieceOnly(
    // GamePieceMode.CUBE);
    // FasterPreloadedPieceThenSimpleBalance cubeThenBalance = new
    // FasterPreloadedPieceThenSimpleBalance(
    // GamePieceMode.CUBE);
    // FasterPreloadedPieceThenSimpleBalance coneThenBalance = new
    // FasterPreloadedPieceThenSimpleBalance(
    // GamePieceMode.CONE);
    // DoNothing nothing = new DoNothing();
    // ChargingSimple xd2 = new ChargingSimple();
    //
    // m_chooser.setDefaultOption("Example path", followExamplePath);
    // m_chooser.addOption("Charging", xd2);
    // m_chooser.addOption("Do nothing", nothing);
    // m_chooser.addOption("Cone Move", coneThenMove);
    // m_chooser.addOption("Cube Move", cubeThenMove);
    // m_chooser.addOption("Cone Balance", coneThenBalance);
    // m_chooser.addOption("Cube Balance", cubeThenBalance);
    // m_chooser.addOption("Cone Only", coneThenStayStill);
    // m_chooser.addOption("Cube Only", cubeThenStayStill);
    SwerveAutos.generateAutos();
    SwerveAutos2.generateAutos();

    Command cubeCharge = new LeavePiece(GamePieceMode.CUBE, TransportTarget.MIDDLE_ROW_BACK)
        .andThen(Commands.parallel(new AutoBalance2Fwd(), new NewTransportGoTo(TransportTarget.RESTING)));
    Command coneCharge = new LeavePiece(GamePieceMode.CONE, TransportTarget.MIDDLE_ROW_BACK)
        .andThen(Commands.parallel(new AutoBalance2Fwd(), new NewTransportGoTo(TransportTarget.RESTING)));

    Command leaveCommunityThenBalance = SwerveAutos2.mobilityAuto.andThen(new AutoBalance2Bwd());

    m_chooser.setDefaultOption("NADA", new DoNothing());
    m_chooser.addOption("CubeRest", new LeavePieceThenRest(GamePieceMode.CUBE, TransportTarget.MIDDLE_ROW_BACK));
    m_chooser.addOption("ConeRest", new LeavePieceThenRest(GamePieceMode.CONE, TransportTarget.MIDDLE_ROW_BACK));
    m_chooser.addOption("CubeFwd", SwerveAutos2.cubeMidFwdAuto.withTimeout(13.5));
    m_chooser.addOption("ConeFwd", SwerveAutos2.coneMidFwdAuto.withTimeout(13.5));
    m_chooser.addOption("CubeCharge", cubeCharge);
    m_chooser.addOption("ConeCharge", coneCharge);
    m_chooser.addOption("El mamalón", SwerveAutos2.twoPieceFreeAuto);
    m_chooser.addOption("MobilityBalance (Untested)", leaveCommunityThenBalance);
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
    // MONTERREY
    // FasterPreloadedPieceOnly xd = new
    // FasterPreloadedPieceOnly(GamePieceMode.CUBE);
    // ChargingSimple xd2 = new ChargingSimple();
    // return xd.andThen(xd2);
    // return new LeavePieceThenRest(GamePieceMode.CONE,
    // TransportTarget.TOP_ROW_BACK);
    // return new LeavePieceThenExitCommunity(GamePieceMode.CUBE,
    // TransportTarget.TOP_ROW_BACK);
    // return new SwerveDriveDistanceFwd(1.0, 1.5, 0.5, true);

    // return new SwerveDriveDistanceFwd(1.0, 4.2, 1.0, true);
    // return new AutoBalance2Fwd();

    // return SwerveAutos.LmaoAuto;

    // return new DoNothing();
    // JALO return new ResetOdometryZeros().andThen(new SwerveDriveDistanceFwd(1.0,
    // 3.0, 0.05, true));

    // return SwerveAutos2.twoPieceFreeAuto;

    // mira, este de charge medio jaló. For reference, es el balance que jaló en
    // frente de toni.
    /*
     * Command auto = new LeavePiece(GamePieceMode.CUBE,
     * TransportTarget.MIDDLE_ROW_BACK)
     * .andThen(Commands.parallel(new AutoBalance2Fwd(), new
     * NewTransportGoTo(TransportTarget.RESTING)));
     * return auto;
     */
    // UNCOMMENT THE FOLLOWING LINE
    return m_chooser.getSelected();
  }

  public Navx getNavx() {
    return navx;
  }

  public SwerveDriveTrain getSwerveDriveTrain() {
    return swerveDriveTrain;
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

  public void setShouldManualHaveLimits(boolean shouldHaveLimits) {
    shouldManualHaveLimits = shouldHaveLimits;
  }

  public boolean getShouldManualHaveLimits() {
    return shouldManualHaveLimits;
  }

  public void setShouldExtensorBeLimitedManual(boolean shouldBeSlower) {
    shouldExtensorBeSlowerInManual = shouldBeSlower;
  }

  public boolean getShouldExtensorBeSlowerInManual() {
    return shouldExtensorBeSlowerInManual;
  }
}

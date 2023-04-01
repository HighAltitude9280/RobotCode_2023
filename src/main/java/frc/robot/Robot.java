// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    // getRobotContainer().getDriveTrain().resetOdometry();
    getRobotContainer().configureButtonBindings();

    getRobotContainer().generateAutos();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    getRobotContainer().getNavx().run();

    debugStringSmartDashboard("Current Game Piece Mode",
        getRobotContainer().getCurrentGamePieceMode().toString());
    // debugStringSmartDashboard("Game Piece Mode",
    // getRobotContainer().getCurrentGamePieceMode().toString());
    // debugBooleanSmartDashboard("Should Manual Have Limits",
    // getRobotContainer().getShouldManualHaveLimits());
    // debugNumberSmartDashboard("Pitch", getRobotContainer().getNavx().getPitch());
    debugNumberSmartDashboard("Yaw", getRobotContainer().getNavx().getYaw());
    // debugNumberSmartDashboard("Roll", getRobotContainer().getNavx().getRoll());
    // debugNumberSmartDashboard("AngAccRoll",
    // getRobotContainer().getNavx().getAngularAccelerationPitch());
    // debugNumberSmartDashboard("AngVelRoll",
    // getRobotContainer().getNavx().getYVel());
    // debugNumberSmartDashboard("AngAccPitch",
    // getRobotContainer().getNavx().getAngularAccelerationPitch());

    // debugBooleanSmartDashboard("Extensor is slower",
    // getRobotContainer().getShouldExtensorBeSlowerInManual());

    // SmartDashboard.putData(getRobotContainer().getWrist());
    // SmartDashboard.putData(getRobotContainer().getArm());
    // SmartDashboard.putData(getRobotContainer().getExtensor());
    // SmartDashboard.putData(getRobotContainer().getDriveTrain());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(false);
    getRobotContainer().getSwerveDriveTrain().recalculateModuleDirections();
  }

  @Override
  public void disabledPeriodic() {
    getRobotContainer().putAutoChooser();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(true);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(true);

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public static void debugPrint(String s) {
    if (HighAltitudeConstants.DEBUG)
      System.out.println(s);
  }

  public static void debugNumberSmartDashboard(String key, double value) {
    if (HighAltitudeConstants.DEBUG)
      SmartDashboard.putNumber(key, value);
  }

  public static void debugStringSmartDashboard(String key, String value) {
    if (HighAltitudeConstants.DEBUG)
      SmartDashboard.putString(key, value);
  }

  public static void debugBooleanSmartDashboard(String key, boolean value) {
    if (HighAltitudeConstants.DEBUG)
      SmartDashboard.putBoolean(key, value);
  }

  public static RobotContainer getRobotContainer() {
    return robotContainer;
  }

  boolean goingToLose() {
    return false;
  }

  void dont() {

  }
}

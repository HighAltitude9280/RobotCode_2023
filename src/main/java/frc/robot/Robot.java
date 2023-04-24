// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

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
    PathPlannerServer.startServer(5811);
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

    SmartDashboard.putString("Current Game Piece Mode",
        getRobotContainer().getCurrentGamePieceMode().toString());
    debugNumberSmartDashboard("Yaw", getRobotContainer().getNavx().getYaw());
    SmartDashboard.putBoolean("ManualLimits?", getRobotContainer().getShouldManualHaveLimits());
    // SmartDashboard.putData(getRobotContainer().getSwerveDriveTrain());
    // SmartDashboard.putData(getRobotContainer().getArm());

    Robot.debugNumberSmartDashboard("Roll", getRobotContainer().getNavx().getRoll());
    Robot.debugNumberSmartDashboard("Roll Velocity", getRobotContainer().getNavx().getYVel());
    Robot.debugNumberSmartDashboard("Roll Acceleration", getRobotContainer().getNavx().getAngularAccelerationRoll());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
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
    getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(true);
    // getRobotContainer().getSwerveDriveTrain().resetPose(new Pose2d());
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous comand (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    getRobotContainer().getLimeLightVision().addLimeLightPoseToOdometry();
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
  public void teleopExit() {
    // UNCOMMENT FOR COAST
    // getRobotContainer().getSwerveDriveTrain().setModulesBrakeMode(false);
    // Only time the modules will be in brake is when exiting teleop
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

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

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
import frc.robot.resources.components.Navx;
import frc.robot.resources.math.splines.CubicSpline;
import frc.robot.resources.math.splines.SplineGenerator;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  private Command m_autoCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Navx navx;
  private DriveTrain driveTrain;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    navx = new Navx();

    driveTrain = new DriveTrain();
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() 
  {
    driveTrain.setDefaultCommand(new DefaultDrive());
    
    OI.getInstance().ConfigureButtonBindings();
  }

  public void generateAutos()
  {
    CubicSpline examplePath = SplineGenerator.generateNaturalSpline(Paths.examplePathControlPoints);
    SplineMove followExamplePath = new SplineMove(examplePath, 0.5, true, false, false, false);
    
    m_chooser.setDefaultOption("Example path", followExamplePath);
    
    m_autoCommand = followExamplePath;
  }

  public void putAutoChooser()
  {
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

  public Navx getNavx()
  {
    return navx;
  }
  public DriveTrain getDriveTrain()
  {
    return driveTrain;
  }
}

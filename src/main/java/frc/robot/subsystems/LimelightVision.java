// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LimelightVision extends SubsystemBase {
  NetworkTable table;

  /** Creates a new LimelightVision. */
  public LimelightVision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    Robot.debugPrint("Target available?: " + getTv());
    // This method will be called once per scheduler run
  }

  public double getTv() {
    return table.getEntry("tv").getDouble(0);
  }

  public double getTx() {
    return table.getEntry("tx").getDouble(0);
  }
}

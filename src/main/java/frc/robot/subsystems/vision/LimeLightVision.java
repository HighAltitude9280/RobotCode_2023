// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;

public class LimeLightVision extends SubsystemBase {
  NetworkTable table;

  /** Creates a new LimelightVision. */
  public LimeLightVision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // Robot.debugPrint("Target available?: " + getTv());
    // This method will be called once per scheduler run

  }

  public void addLimeLightPoseToOdometry() {
    if (getPipeline() == 0 && getTv() == 1 && HighAltitudeConstants.USING_VISION_FOR_POSE) {
      Robot.getRobotContainer().getSwerveDriveTrain().addVisionMeasurement(
          getPose(true),
          Timer.getFPGATimestamp() - (getTl() / 1000.0) - (getCl() / 1000.0));
    }
  }

  public double getTv() {
    return table.getEntry("tv").getDouble(0);
  }

  public double getTx() {
    return table.getEntry("tx").getDouble(0);
  }

  public double getTy() {
    return table.getEntry("ty").getDouble(0);
  }

  public double getTa() {
    return table.getEntry("ta").getDouble(0);
  }

  public double getTl() {
    return table.getEntry("tl").getDouble(0);
  }

  public double getCl() {
    return table.getEntry("cl").getDouble(0);
  }

  public double getPipeline() {
    return table.getEntry("pipeline").getDouble(0);
  }

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public Pose2d getPose(boolean usingAllianceColor) {
    double[] poseData;
    Pose3d measuredPose;

    if (usingAllianceColor) {
      if (DriverStation.getAlliance() == Alliance.Red)
        poseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired")
            .getDoubleArray(new double[6]);
      else
        poseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);
    } else
      poseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
          .getDoubleArray(new double[6]);

    measuredPose = new Pose3d(poseData[0], poseData[1], poseData[2],
        new Rotation3d(poseData[3], poseData[4], poseData[5]));

    return measuredPose.toPose2d();
  }
}
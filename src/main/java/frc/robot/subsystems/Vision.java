// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  PhotonCamera photonCamera;
  RobotPoseEstimator robotPoseEstimator;

  double fieldLength = Units.feetToMeters(54);
  double filedWidth = Units.feetToMeters(27);

  boolean hasTargets = false;
  private double previusPipelineTimestamp = 0;

  /** Creates a new Vision. */
  public Vision() {
    photonCamera = new PhotonCamera("Solis");
    final AprilTag tag18 = new AprilTag(
        18,
        new Pose3d(
            new Pose2d(
                fieldLength,
                filedWidth / 2.0,
                Rotation2d.fromDegrees(180))));
    final AprilTag tag01 = new AprilTag(
        01,
        new Pose3d(
            new Pose2d(
                0.0, filedWidth / 2.0,
                Rotation2d.fromDegrees(0.0))));

    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag18);
    atList.add(tag01);

    // TODO - once 2023 happens, replace this with just loading the 2023 field
    // arrangement
    AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, fieldLength, filedWidth);
  }

  @Override
  public void periodic() {

    var pipelineResult = photonCamera.getLatestResult();
    double resultTimestamp = pipelineResult.getTimestampSeconds();

    if (resultTimestamp != previusPipelineTimestamp && pipelineResult.hasTargets()) {
      previusPipelineTimestamp = resultTimestamp;

      var target = pipelineResult.getBestTarget();
      var yaw = target.getYaw();
      var pitch = target.getPitch();
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();

      if (poseAmbiguity <= 0.2 && targetID >= 0) {
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
      }
    }

    // This method will be called once per scheduler run
  }
}

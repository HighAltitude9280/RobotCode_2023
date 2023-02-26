// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCameras extends SubsystemBase {
  UsbCamera camera;

  /** Creates a new DriverCameras. */
  public DriverCameras() {
    camera = CameraServer.startAutomaticCapture(0);
    camera.setVideoMode(PixelFormat.kMJPEG, 640, 360, 30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

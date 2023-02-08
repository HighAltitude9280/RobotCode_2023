// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;

public class DriverCameras extends SubsystemBase {

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;

  /** Creates a new Bision. */
  public DriverCameras() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setVideoMode(PixelFormat.kMJPEG, 640, 360, 30);

    camera2 = CameraServer.startAutomaticCapture(1);
    camera2.setResolution(176, 144);
    camera2.setVideoMode(PixelFormat.kMJPEG, 176, 144, 7);

    Robot.debugPrint(camera1.getName());
    Robot.debugPrint(camera2.getName());

    server = CameraServer.getServer();

    // camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

  }

  @Override
  public void periodic() {

    if (OI.getInstance().getPilot().getButtonObj(ButtonType.B).getAsBoolean()) {
      System.out.println("Setting camera 2");
      server.setSource(camera2);
    } else {
      System.out.println("Setting camera 1");
      server.setSource(camera1);
    }

    // This method will be called once per scheduler run
  }
}

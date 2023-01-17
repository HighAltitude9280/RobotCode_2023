// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.PWMLEDStrip;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  OldHighAltitudePWMLEDStrip leds;

  /** Creates a new LEDs. */

  public LEDs() {
    leds = new OldHighAltitudePWMLEDStrip(0, 59);
  }

  public void allLedsOff() {
    leds.allLedsOff();
  }

  public void setRGB(int r, int g, int b) {
    leds.setSolidRGB(r, g, b);
  }

  public void setFireAnimation(int hue) {
    leds.setBasicFire(hue, 255, 2, 200);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

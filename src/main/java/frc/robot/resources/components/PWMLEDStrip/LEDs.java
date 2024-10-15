// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.PWMLEDStrip;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  HighAltitudePWMLEDStrip leds;

  /** Creates a new LEDs. */

  public LEDs() {
    leds = new HighAltitudePWMLEDStrip(9, 140);
    leds.allLedsOff();
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

  public void setFireAnimation(int hue, int saturation, int cooling, int sparking) {
    leds.setBasicFire(hue, saturation, cooling, sparking);
  }

  public void setCoolerFireAnimationWithInput(int hue, double in, double mn, double mx) {
    leds.setFireWithVariableIntensity(hue, in, mn, mx, 50, 0, false);
    leds.setFireWithVariableIntensity(hue, in, mn, mx, 50, 30, true);
  }

  @Override
  public void periodic() {
    // leds.setBasicFire(64, 225, 2, 200);
    // leds.setRhythmSingleHue(0, 0.5);
    // leds.setRhythmSingleHueDouble(0, 0.5, 70, 0, false);
    // leds.setRhythmSingleHueDouble(0, 0.5, 70, 70, true);
    // leds.setRainbowCycle();
    // This method will be called once per scheduler run
  }
}

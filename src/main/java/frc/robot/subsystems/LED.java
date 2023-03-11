// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  public class RGB {
    public int R,G,B;
    public RGB(int R, int G, int B) {
      this.R = R;
      this.G = G;
      this.B = B;
    }
  }

  public RGB purple = new RGB(100, 0, 200);
  public RGB yellow = new RGB(180, 230, 75);
  public RGB orange = new RGB(245, 90, 5);

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private final int LED_LEN = 10;

  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(LED_LEN);

    led.setLength(buffer.getLength());
    setOrange();
  }

  public void setRGB(int R, int G, int B) {
    for (int i=0;i<buffer.getLength();++i) {
        buffer.setRGB(i, R, G, B);
    }

    led.setData(buffer);
    led.start();
  }

  public void setRGB(RGB rgb) {
    for (int i=0;i<buffer.getLength();++i) {
        buffer.setRGB(i, rgb.R, rgb.G, rgb.B);
    }

    led.setData(buffer);
    led.start();
  }

  public void reset() {
    setRGB(0,0,0);
  }

  public void setCube() {
    setRGB(purple);
  }

  public void setCone() {
    setRGB(yellow);
  }

  public void setOrange() {
    setRGB(orange);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

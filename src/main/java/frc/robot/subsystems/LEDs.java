// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  /** Creates a new LEDS. */
  public LEDs() {
    m_led = new AddressableLED(3);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.start();

  }

  public void setColorLED() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 032, 564, 543);
    }
  }
  public void whatLightLED() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, Color.kBlueViolet);
    }
  }
  public void royalBlueLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 32, 135);
    }
  }
  public void orangeLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 127, 0);
    }
  }
  public void yellowLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 230, 223, 0);
    }
  }
  public void greenLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 250, 0);
    }
  }
  public void redLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 250, 0, 0);
    }
  }
}
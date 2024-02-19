// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  
  /** Creates a new LEDS. */
  public LEDs() {
    m_led = new AddressableLED(LEDConstants.LED_PORT_ID);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LED);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
  public void royalBlueLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 59, 174);
    } m_led.setData(m_ledBuffer);
  }
  public void orangeLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 43, 0);
    }  m_led.setData(m_ledBuffer);
  }
  public void yellowLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 230, 223, 0);
    }  m_led.setData(m_ledBuffer);
  }
  public void greenLED() {
    for (var i = 0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 250, 0);
    } m_led.setData(m_ledBuffer);
  }
  public void redLED() {
    for (var i=0; i< m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 250, 0, 0);
    } m_led.setData(m_ledBuffer);
  }
  public void RGBColor(int r, int g, int b){
    for (var i=0; i< m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
  } m_led.setData(m_ledBuffer);
  }

  public void rainbowLED() {
    int length = m_ledBuffer.getLength();
    int color = 0;
    while (true) {
        color = (color + 1) % 256;
        for (var i = 0; i < length; i++) {
            m_ledBuffer.setRGB(i, color, 0, 0); // Измените значения R, G и B, чтобы создать разноцветный эффект
        }
        m_led.setData(m_ledBuffer);
        try {
            Thread.sleep(100); // Подождите некоторое время перед обновлением цвета
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
      }
  }
}

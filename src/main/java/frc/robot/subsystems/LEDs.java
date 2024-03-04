// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import java.lang.Thread;

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

    public void noLED() {
      for (var i=0; i< m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
      } m_led.setData(m_ledBuffer);
    }

    public void blinkingColor(int r, int g, int b) {
      int length = m_ledBuffer.getLength();
      for (int i = 0; i<length; i++) {
        m_ledBuffer.setRGB(i, r, g, b);
      }
      m_led.setData(m_ledBuffer);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      for (int i = 0; i<length; i++) {
        m_ledBuffer.setRGB(i , 0, 0, 0);
      }
      m_led.setData(m_ledBuffer);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }


  // public void color(int r, int g, int b){
  //   for(var i=0; i<m_ledBuffer.getLength();i++){
  //     m_ledBuffer.setRGB(i, r, g, b);
  //   }
  // }
  //   public void blinkingRed() {
  //   int length = m_ledBuffer.getLength();
  //   int blue = 20;
  //   int red = 200;
  //   int green = 20;
  //   while (true) {
  //       blue = (blue - 5) % 10;
  //       red = (red + 500) % 600;
  //       green = (green - 5) % 10;
  //       for (var i = 0; i < length; i++) {
  //           m_ledBuffer.setRGB(i, red, 0, 0); 
  //       }
  //       m_led.setData(m_ledBuffer);
  //       try {
  //           Thread.sleep(100); 
  //       } catch (InterruptedException e) {
  //           e.printStackTrace();
  //       }
  //     }
  //   }
  
  //   public void blinkingGreen() {
  //   int length = m_ledBuffer.getLength();
  //   int blue = 20;
  //   int red = 20;
  //   int green = 200;
  //   while (true) {
  //       blue = (blue - 5) % 10;
  //       red = (red - 5) % 10;
  //       green = (green + 500) % 600;
  //       for (var i = 0; i < length; i++) {
  //           m_ledBuffer.setRGB(i, 0, green, 0);
  //       m_led.setData(m_ledBuffer);
  //       try {
  //           Thread.sleep(100); 
  //       } catch (InterruptedException e) {
  //           e.printStackTrace(); 
  //       }
  //     }
  //   }
  // }

  // public void blinkingYellow() {
  //   int length = m_ledBuffer.getLength();
  //   int blue = 10;
  //   int red = 200;
  //   int green = 100;
  //   while (true) {
  //       blue = (blue - 10) % 20;
  //       red = (red + 500) % 600;
  //       green = (green + 500) % 600;
  //       for (var i = 0; i < length; i++) {
  //           m_ledBuffer.setRGB(i, 230, 223, blue);
  //       m_led.setData(m_ledBuffer);
  //       try {
  //           Thread.sleep(200); 
  //       } catch (InterruptedException e) {
  //           e.printStackTrace();
  //       }
  //     }
  //   }
  // }

  //   public void blinkOrange() {
  //   int length = m_ledBuffer.getLength();
  //   int blue = 20;
  //   int red = 20;
  //   int green =100;
  //   while (true) {
  //       blue = (blue - 10) % 20;
  //       red = (red + 255) % 510;
  //       green = (green - 44) % 80;
  //       for (var i = 0; i < length; i++) {
  //           m_ledBuffer.setRGB(i, red, green, 0);
  //       m_led.setData(m_ledBuffer);
  //       try {
  //           Thread.sleep(50); 
  //       } catch (InterruptedException e) {
  //           e.printStackTrace();
  //       }
  //     }
  //   }
  // }
}
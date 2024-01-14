// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;


  /** Creates a new LED. */
  public LED(int PWMPort) {
    // Set PWM port for AddressableLED
    m_led = new AddressableLED(PWMPort);
    m_ledBuffer = new AddressableLEDBuffer(30);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void SetColorRGB(int index, int r, int g, int b){
    m_ledBuffer.setRGB(index, r, g, b);

    m_led.setData(m_ledBuffer);
  }

  public void SetColorHSV(int index, int h, int s, int v){
    m_ledBuffer.setHSV(index, h, s, v);

    m_led.setData(m_ledBuffer);
  }
 
  public void SetEntireStripColorRGB(int r, int g, int b)
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }   

    m_led.setData(m_ledBuffer);
  }

  public void SetEntireStripColorHSV(int h, int s, int v)
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }   

    m_led.setData(m_ledBuffer);
  }
  
  public int getStripLength()
  {
    return m_ledBuffer.getLength();  
  }

  public void setHalfStripColorRGB(int r, int g, int b, int part)
  {
    int strip_length = m_ledBuffer.getLength();
    int offset = 0;
    if(part == 1){
      offset = (strip_length / 2) - 1;
      offset = (strip_length / 2) - 1;
    }
    for (var i = 0; i < (strip_length / 2); i++) {
      m_ledBuffer.setRGB(i + offset, r, g, b);
    }   
       
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

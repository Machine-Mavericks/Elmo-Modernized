// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDBlinkin extends SubsystemBase {

  //Victor led;
  PWM led;
  /** Creates a new LEDBlinkin. */
  public LEDBlinkin() {

      // set up pwm channel
      //led = new Victor(RobotMap.PWMPorts.LED_BLINKIN);
      led = new PWM(RobotMap.PWMPorts.LED_BLINKIN);
      
      setPattern(LED_PATTERN.OFF);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum LED_PATTERN {
    OFF,
    REDBALL,
    BLUEBALL,
    HUB,
    LOWBATTERY,
    DISCO
  };

  // sets pattern of LED strip
  public void setPattern(LED_PATTERN pattern)
  {
    switch (pattern) {
      case OFF:
        led.setSpeed(0.99);    // black
      break;
      case REDBALL:
        led.setSpeed(-0.11);   // strobe red
      break;
      case BLUEBALL:
        led.setSpeed(-0.09);   // strobe blue
      break;
      case HUB:
        led.setSpeed(-0.07);   // strobe red
      break;
      case LOWBATTERY:
        led.setSpeed(0.91);    // solid purple
      break;
      case DISCO:
        led.setSpeed(-0.45);   // color wave - rainbow
      break;

    }
    
  }


}

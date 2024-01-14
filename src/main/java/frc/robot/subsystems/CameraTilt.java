// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class CameraTilt extends SubsystemBase {

  // create servo
  Servo m_servo = new Servo(RobotMap.PWMPorts.CAMERA_SERVO_ID);

  public GenericEntry m_cameraTiltAngle;

  double m_currentangle=0.45;


  /** Creates a new CameraTilt. */
  public CameraTilt() {
    initializeShuffleboard();
  
    // set default camera angle -14.4deg
    setAngle(-14.4);
  }

  @Override
  public void periodic() {
    // go ahead and set camera angle
    m_servo.set(m_currentangle);
    
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  // sets camera tilt to desired angle
  // Value is angle (deg)
  public void setAngle(double angle)
  {
      // convert -90 to 90deg range to 0 to 1.0 range
      double value = 0.5 + angle * 0.00555;
    
      // limit value to allowable range by camera
      if (value > 0.25)  
        value = 0.25;
      if (value <0.10)    // was 0.28    // 0.34 works well w/o camera cropping
        value = 0.10;

      m_currentangle = value;
  }

  /** returns current camera angle (in deg) */
  public double getAngle()
  {
    return ((m_currentangle - 0.5) * 180.0);
  }

  
  // -------------------- Shuffleboard Commands --------------------

  /** Initialize subsystem shuffleboard page and controls */
private void initializeShuffleboard() { 
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Camera Tilt");

    ShuffleboardLayout l1 = Tab.getLayout("Camera Tilt", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(2, 4);
    m_cameraTiltAngle = l1.add("Angle(deg)", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_cameraTiltAngle.setDouble(getAngle());
  }
}

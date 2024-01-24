// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// libraries needed for NavX
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  // subsystem shuffleboard controls
  private GenericEntry m_gyroPitch;
  private GenericEntry m_gyroYaw;
  private GenericEntry m_gyroRoll;
  private GenericEntry m_xAcceleration;
  private GenericEntry m_yAcceleration;
  
  // make our gyro object
  private AHRS gyro;

  /** Creates a new Gyro. */
  public Gyro() {
    //gyro = new AHRS(Port.kMXP);
    gyro = new AHRS(Port.kMXP);
    gyro.reset();
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  /**
   * Gets the yaw of the robot
   * 
   * @return current yaw value (-180 to 180)
   */
  public double getYaw() {
    // Flip angle since gyro is mounted upside down
    return gyro.getYaw();  
  }

  /**
   * Gets the pitch of the robot
   * 
   * @return current pitch value (-180 to 180)
   */
  public double getPitch() {
    return gyro.getPitch();
  }

  /**
   * Resets yaw to zero
   */
  public void resetGyro() {
    
    // reset our Gyro
    gyro.reset();
  }


  /**
   * Accumulated yaw
   * 
   * @return accumulated angle in degrees
   */
  public double continuousYaw() {
    return gyro.getAngle();
  }

  /** 
   * Get Roll
   * 
   * @return -180 to 180 degrees
   */
  public double getRoll() {
    return gyro.getRoll();
  }

  /**
   * X Acceleration
   * 
   * @return ratio of gravity
   */
  public double getXAcceleration() {
    return gyro.getRawAccelX();
  }

  /**
   * Y Acceleration
   * 
   * @return ratio of gravity
   */
  public double getYAcceleration() {
    return gyro.getRawAccelY();
  }

  

  /** Gyro Shuffleboard */

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Gyroscope");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Gyroscope", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_gyroPitch = l1.add("Pitch (deg)", 0.0).getEntry();
    m_gyroYaw = l1.add("Yaw (deg)", 0.0).getEntry();
    m_gyroRoll = l1.add("Roll (deg)", 0.0).getEntry();
    m_xAcceleration = l1.add("X Acceleration", 0.0).getEntry();
    m_yAcceleration = l1.add("Y Acceleration", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_gyroPitch.setDouble(getPitch());
    m_gyroYaw.setDouble(getYaw());
    m_gyroRoll.setDouble(getRoll());
    m_xAcceleration.setDouble(getXAcceleration());
    m_yAcceleration.setDouble(getYAcceleration());
  }
}
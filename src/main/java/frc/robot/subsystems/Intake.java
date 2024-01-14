// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private GenericEntry m_speed;
  private GenericEntry m_speedslider;
  public GenericEntry m_limitSwitch;

  private TalonFX intakeFalcon = new TalonFX(RobotMap.CANID.INTAKE_FALCON);
  
  private DigitalInput ballInputSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_ID);
  
  public static double MOTORSPEED = 0.30;

  // Convert pulses per 100ms to rpm 
  // Multiply by 10 to get pulses/second
  // Divide by 2048, motor has 2048 pulses in a revolution, we now have the number of rotations per second
  // Multiply by 60 to get rpm
  public static double MOTORSPEEDCONVERSION = (10.0/2048.0)*60.0;

  /** Creates a new Intake. */
  public Intake() {
    intakeFalcon.set(0);
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }
/**
 * This function will set the falcon motor speed to whatever double is passed in
 * @param motorSpeed percentage motor power, ranging from 0.0 to 1.0
 */
  public void setMotorSpeed(double motorSpeed) {
    intakeFalcon.set(motorSpeed);
  }


  /** Get intake motor speed in rpm */
  public double getMotorSpeed() {
    return intakeFalcon.getVelocity().getValue() * MOTORSPEEDCONVERSION;
  }


  /** Get Ball Input Limit Switch Status
   * Returns true if limit switch actuated
   * False otherwise
   */
  public boolean GetIntakeLimitSwitchStatus()
  {
    return !ballInputSwitch.get();
  }

    // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create Intake page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Intake");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Intake", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_speed = l1.add("Speed", 0.0).getEntry();

    m_speedslider = Tab.add("Speed", MOTORSPEED).withWidget(BuiltInWidgets.kNumberSlider)
                                                .withPosition(2, 0)
                                                .withSize(4, 1)
                                                .withProperties(Map.of("min", 0, "max", 1))
                                                .getEntry();

    // does camera detect target
    m_limitSwitch = Tab.add("SwitchActivated", false).withPosition(3,0).getEntry();
  }

  /** Update subsystem shuffle board page with current Intake values */
  private void updateShuffleboard() {
    // write current intake data
    m_speed.setDouble(getMotorSpeed());
    MOTORSPEED = m_speedslider.getDouble(0.0);
    m_limitSwitch.setBoolean (GetIntakeLimitSwitchStatus());
  }
}

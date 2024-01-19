// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
 
  // shuffleboard controls
  GenericEntry forwardLimit;
  GenericEntry reverseLimit;
  GenericEntry motorPosition;
  GenericEntry encoderValue;
  GenericEntry ChosenSpeed;

  public TalonFX m_climberFalcon; 
  final VelocityVoltage m_velocityControl = new VelocityVoltage(0);

  public Climber() {
    // create falcon motors
    m_climberFalcon = new TalonFX(RobotMap.CANID.CLIMBER_FALCON);
  
    m_climberFalcon.setPosition(0);
    m_climberFalcon.setInverted(true);
      
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    var slot0Configs = new Slot0Configs();
    //slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.09;
    //slot0Configs.kI = 0.48;
    slot0Configs.kD = 0.0;

    talonConfig.Slot0 = slot0Configs;

    m_climberFalcon.getConfigurator().apply(talonConfig);

    // m_climberFalcon.configForwardSoftLimitEnable(true,0);
    // m_climberFalcon.configForwardSoftLimitThreshold(8.80*48*2048.0);

    // right motor spins in opposite direction
    
    // set PID gains
    //m_climberFalcon.config_kF(0, 0.044, 0);
    //m_climberFalcon.config_kP(0, 0.09, 0);
    //m_climberFalcon.config_kI(0, 0.00010, 0); 

    // reset encoder positions
    

    // talonConfig.configPeakOutputForward(1, 0);
    // m_climberFalcon.configPeakOutputReverse(-1.0, 0);


    

    // initialize shuffleboard
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  /** set motor posiiton */
  public void motorVelocity()
  {  
    m_velocityControl.Slot = 0;
    //m_climberFalcon.setControl(m_velocityControl.withVelocity(ChosenSpeed.getDouble(50) / 60));
    // ChosenSpeed.getDouble(3000)* (2048 / 600.0) 
    
  }
  /** stops the climbing motor*/
  public void stopMotor() {
    m_climberFalcon.setControl(m_velocityControl.withVelocity(0));
  }
 
  public double encoderVal() {
    return (m_climberFalcon.getPosition().getValue());
  }


  public GenericEntry climbSpeed;
  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
   // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("climber");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    forwardLimit = l1.add(" fwd limit", 0.0).getEntry();
    reverseLimit = l1.add(" rev limit", 0.0).getEntry();
  
    ShuffleboardLayout l2 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l2.withPosition(2, 0);
    l2.withSize(1, 2);
    motorPosition = l1.add(" mtr pos", 0.0).getEntry();

    ShuffleboardLayout l3 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l3.withPosition(4, 0);
    l3.withSize(1, 2);
    encoderValue = l3.add("Climbing Motor Encoder Reading", 0.0).getEntry();

    ChosenSpeed = Tab.addPersistent("cimber speed", 3000)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",3000, "max", 6000))
        .getEntry();
  }

  /** Update subsystem shuffle board page with climber values */
  private void updateShuffleboard() {
    encoderValue.setDouble(m_climberFalcon.getPosition().getValue());
  }
}

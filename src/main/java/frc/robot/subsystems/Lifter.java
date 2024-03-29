// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Lifter extends SubsystemBase {

  public GenericEntry limitSwitch;
  public GenericEntry lifterSpeed;
  //two talon
  public TalonFX lifterFalcon = new TalonFX(RobotMap.CANID.LIFTER_FALCON);

  public GenericEntry lifterSpeedEntry;

  public DigitalInput liftLimit = new DigitalInput(RobotMap.LIFTER_LIMIT_ID);

  public int shotsTaken = 0;
  private boolean hasBall = false;
  public boolean shooting = false;

  private VelocityDutyCycle m_velocityControl = new VelocityDutyCycle(0);

  /** Creates a new Lifter. */
  public Lifter() {
    lifterFalcon.setNeutralMode(NeutralModeValue.Brake);


    TalonFXConfiguration lifterConfiguration = new TalonFXConfiguration();
    lifterConfiguration.Slot0.kV = 0.0088;
    lifterConfiguration.Slot0.kD = 0.0;
    lifterConfiguration.Slot0.kP = 0.0120;
    lifterConfiguration.Slot0.kI = 0.0200;
    
    lifterFalcon.getConfigurator().apply(lifterConfiguration);

    // lifterFalcon.config_kF(0, 0.044, 0);
    // lifterFalcon.config_kD(0, 0.0, 0);
    // lifterFalcon.config_kP(0, 0.06, 0);
    // lifterFalcon.config_kI(0, 0.00010, 0); 
    // lifterFalcon.config_kD(0, 0.05, 0);
    // lifterFalcon.configMaxIntegralAccumulator(0, 120000.0, 0);

    // rightShooterFalcon.set(ControlMode.PercentOutput, 0);

    // TODO: We probably don't need these right?
    // lifterFalcon.configPeakOutputForward(1, 0);
    // lifterFalcon.configPeakOutputReverse(-1.0, 0);

    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (hasBall && liftLimit.get() && OI.shootButton.getAsBoolean()){
      hasBall = false;
      shotsTaken++;
    }
    if (!liftLimit.get()){
      hasBall = true;
    }
    SmartDashboard.putNumber("ShotsTaken", shotsTaken);

    updateShuffleboard();
  }

  public void liftBalls(){
    lifterFalcon.setControl(m_velocityControl.withSlot(0).withVelocity(RobotMap.BALL_LIFTER_SPEED / 60)); 
  }

  public void liftBalls(double Speed){
    lifterFalcon.setControl(m_velocityControl.withSlot(0).withVelocity(Speed / 60)); 
  }

  public void releaseBalls(){ // BALL_LIFTER_SPEED is in RPM
    lifterFalcon.setControl(m_velocityControl.withSlot(0).withVelocity(-RobotMap.BALL_LIFTER_SPEED / 60)); 
    // -RobotMap.BALL_LIFTER_SPEED* (2048 / 600.0)
  }

  public void stopMotor() {
    lifterFalcon.setControl(m_velocityControl.withSlot(0).withVelocity(0));
  }

  public void initializeShuffleboard() {
    ShuffleboardTab Tab = Shuffleboard.getTab("Lifter");
    ShuffleboardLayout l1 = Tab.getLayout("Lifter", BuiltInLayouts.kList);
    l1.withPosition(3, 0);
    l1.withSize(1, 4);
    //limitSwitch = l1.add("limswitch", 0.0).getEntry();
    lifterSpeed = l1.add("lifter speed (RPM)",0.0).getEntry();
    limitSwitch = Tab.add("SwitchActivated", false).withPosition(3,0).getEntry();
  }
  public void updateShuffleboard() {

    limitSwitch.setBoolean(liftLimit.get());
    lifterSpeed.setDouble(lifterFalcon.getVelocity().getValue() * 60);
  }
}

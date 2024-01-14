// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix6.hardware.TalonFX;

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

  /** Creates a new Lifter. */
  public Lifter() {

    lifterFalcon.setNeutralMode(NeutralMode.Brake);

    lifterFalcon.config_kF(0, 0.044, 0);
    lifterFalcon.config_kD(0, 0.0, 0);
    lifterFalcon.config_kP(0, 0.06, 0);
    lifterFalcon.config_kI(0, 0.00010, 0); 
    // lifterFalcon.config_kD(0, 0.05, 0);
    // lifterFalcon.configMaxIntegralAccumulator(0, 120000.0, 0);

    // rightShooterFalcon.set(ControlMode.PercentOutput, 0);
    lifterFalcon.configPeakOutputForward(1, 0);
    lifterFalcon.configPeakOutputReverse(-1.0, 0);

    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (hasBall && liftLimit.get() && OI.shootButton.get()){
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
    lifterFalcon.set(ControlMode.Velocity, RobotMap.BALL_LIFTER_SPEED* (2048 / 600.0)); 
  }

  public void liftBalls(double Speed){
    lifterFalcon.set(ControlMode.Velocity, Speed* (2048 / 600.0)); 
  }

  public void releaseBalls(){
    lifterFalcon.set(ControlMode.Velocity, -RobotMap.BALL_LIFTER_SPEED* (2048 / 600.0)); 
  }

  public void stopMotor() {
    lifterFalcon.set(ControlMode.PercentOutput, 0.0);
    // lifterFalcon.NeutralMode.Brake=(2);

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
    lifterSpeed.setDouble(lifterFalcon.getSelectedSensorVelocity() / (2048 / 600.0));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  // shooter motors
  private TalonFX rightShooterFalcon = new TalonFX(RobotMap.CANID.RIGHT_SHOOTER_FALCON);
  private TalonFX leftShooterFalcon = new TalonFX(RobotMap.CANID.LEFT_SHOOTER_FALCON);
  private TalonFX topShooterFalcon = new TalonFX(RobotMap.CANID.TOP_SHOOTER_FALCON);

  // servo to move shooter hood
  private Servo m_servo = new Servo(RobotMap.PWMPorts.SHOOTER_SERVO_ID);

  //private GenericEntry ChosenSpeed;
  //private GenericEntry ChosenAngle;
  //public GenericEntry ChosenIdleSpeed;
  //public GenericEntry ChosenLifterSpeed;
  private GenericEntry motorSpeed;
  private GenericEntry motorVoltage;
  private GenericEntry rightMotorCurrent;
  private GenericEntry leftMotorCurrent;
  private GenericEntry targetSpeed;
  private GenericEntry topShooterSpeed;
  private GenericEntry topShootertargetSpeed;
  private GenericEntry hoodTargetPos;
  private GenericEntry hoodEstimatedPos;

  // current shooter hood angle setting
  private double m_HoodTargetPos;
  // current estimate of hood angle
  private double m_HoodEstimatedPos;
  // max speed servo can move at = range of position / total time
  // -1.0 to 0.75 over time of 3.2s
  private final double m_HoodMaxSpeed = (1.0 + 0.75)/3.2;


  // Shot evaluation variables
  
  public double HubDistanceOffset = 0;

  //public static int ShotsTaken = 0;
  public static Pose2d RobotPose;
  public int ShotsLogged = 0;

  public String[] LastTwoShots = {"",""};

  //Logs odometry and shot successfullness, but does nothing with it, implement logging to file later.
  public ArrayList<ArrayList<String>> ShotList = new ArrayList<ArrayList<String>>();

  /** Creates a new Shooter. */
  public Shooter() {
    rightShooterFalcon.setInverted(TalonFXInvertType.Clockwise);
    leftShooterFalcon.follow(rightShooterFalcon);
    leftShooterFalcon.setInverted(InvertType.OpposeMaster);

    rightShooterFalcon.configVoltageCompSaturation(11.0, 0);
    leftShooterFalcon.configVoltageCompSaturation(11.0, 0);

    // old PIF gains archived at bottom of file
    rightShooterFalcon.config_kF(0, 0.0477, 0); // 0.047698 (works ok)
    rightShooterFalcon.config_kP(0, 0.28, 0); // 0.35 // 0.6 //0.75 (works ok) // was 0.38
    rightShooterFalcon.config_kI(0, 0.00005, 0); // kI=0.001
    rightShooterFalcon.config_kD(0, 0.0, 0);   // was 0.05
    rightShooterFalcon.configMaxIntegralAccumulator(0, 60000.0, 0);

    rightShooterFalcon.configPeakOutputForward(1, 0);
    rightShooterFalcon.configPeakOutputReverse(0.0, 0);
    
    // setup top flywheel PIDF gains
    topShooterFalcon.config_kF(0, 0.045, 0);
    topShooterFalcon.config_kP(0, 0.18, 0); 
    topShooterFalcon.config_kI(0, 0.001, 0); 
    topShooterFalcon.config_kD(0, 0.0, 0);   
    topShooterFalcon.configMaxIntegralAccumulator(0, 40000.0, 0);
    
    topShooterFalcon.configPeakOutputForward(1.0, 0);
    topShooterFalcon.configPeakOutputReverse(-1.0, 0);
    
    // initialize the shuffleboard
    initializeShuffleboard();

    // set up PWM to operate hood servos
    m_servo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    // set default hood target position and estimate
    m_HoodTargetPos = -0.5;
    m_HoodEstimatedPos = m_HoodTargetPos;
  }

  @Override
  public void periodic() {
    // set hood angle to current target
    m_servo.setSpeed(m_HoodTargetPos);

    // update our internal estimate of hood position
    if (m_HoodEstimatedPos < m_HoodTargetPos)
      m_HoodEstimatedPos = Math.min(m_HoodEstimatedPos + 0.02 * m_HoodMaxSpeed, m_HoodTargetPos);
    if (m_HoodEstimatedPos > m_HoodTargetPos)
    m_HoodEstimatedPos = Math.max(m_HoodEstimatedPos - 0.02 * m_HoodMaxSpeed, m_HoodTargetPos);

    // update shuffleboard
    updateShuffleboard();
  }

  /** This method will set the motors to the given motor speed
   * @param shooterSpeed the desired motor speed in rpm */
  public void setShooterSpeed(double shooterSpeed) {
    rightShooterFalcon.set(ControlMode.Velocity, shooterSpeed * (2048 / 600.0));
  }

  /** This method will return motor speed (rpm)*/
  public double getShooterSpeed() {
    return rightShooterFalcon.getSelectedSensorVelocity() / (2048 / 600.0);
  }

  public double getShooterTargetSpeed() {
    return rightShooterFalcon.getClosedLoopTarget() * ((10.0 / 2048.0) * 60);
  }

  /** This method will set the motors to the given motor speed
   * @param shooterSpeed the desired motor speed in rpm */
  public void setTopShooterSpeed(double shooterSpeed) {
    topShooterFalcon.set(ControlMode.Velocity, -shooterSpeed * (2048 / 600.0));
  }

  /** This method will return motor speed (rpm)*/
  public double getTopShooterSpeed() {
    return -topShooterFalcon.getSelectedSensorVelocity() / (2048 / 600.0);
  }

  public double getTopShooterTargetSpeed()
  {
    return -topShooterFalcon.getClosedLoopTarget() * ((10.0 / 2048.0) * 60);
  }


  // sets camera tilt to desired angle
  // input: actuator setting
  public void setShooterAngle(double angle) {
    // limit hood servo to setting between -1.0 and 0.75 to avoid hardware damage
    if (angle < -1.0)
      angle = -1.0;
    if (angle > 0.75)
      angle = 0.75;
    
    // go ahead and set angle
    m_HoodTargetPos = -1.0; //angle; TODO: for testing only, put this back
  }

  /** Returns current hood servo position  */
  public double getHoodTargetPos() {
    return m_HoodTargetPos;
  }

  /** Returns estimated hood servo position */
  public double getHoodEstimatedPos() {
      return m_HoodEstimatedPos;
  }


  /** Shooter Shuffleboard */

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */

  public void initializeShuffleboard() {
    ShuffleboardTab Tab = Shuffleboard.getTab("Shooter");

    // ChosenIdleSpeed = Shuffleboard.getTab("Shooter")
    //     .add("Idle speed (RPM)", 1.0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", 0, "max", 5000.0))
    //     .getEntry();

    // ChosenLifterSpeed = Shuffleboard.getTab("Shooter")
    //     .add("Lifter speed (pct output)", 1.0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", 0, "max", 1.0))
    //     .getEntry();

    // ChosenSpeed = Tab.add("shooter Speed (RPM)", 1.0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", 0, "max", 5000))
    //     .getEntry();

    // ChosenAngle = Tab.add("Shooter angle", 1.0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", -1.0, "max", 0.75))
    //     .getEntry();

    // Shooter Parameters
    ShuffleboardLayout l1 = Tab.getLayout("Shooter", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 3);
    motorSpeed = l1.add("motor speed", 0.0).getEntry();
    motorVoltage = l1.add("motor voltage", 0.0).getEntry();
    leftMotorCurrent = l1.add("L motor current", 0.0).getEntry();
    rightMotorCurrent = l1.add("R motor current", 0.0).getEntry();
    targetSpeed = l1.add("Target Speed", 0.0).getEntry();

    ShuffleboardLayout l2 = Tab.getLayout("Shooter2", BuiltInLayouts.kList);
    l2.withPosition(1, 0);
    l2.withSize(1, 3);
    topShooterSpeed = l2.add("Top Fly Speed", 0.0).getEntry();
    topShootertargetSpeed = l2.add("Top Fly Target", 0.0).getEntry();
    hoodTargetPos= l2.add("Servo Target", 0.0).getEntry();
    hoodEstimatedPos = l2.add("Esimated Pos", 0.0).getEntry();
  }

  public void updateShuffleboard() {

    // update shooter parameters
    motorSpeed.setDouble(rightShooterFalcon.getSelectedSensorVelocity() * ((10.0 / 2048.0) * 60));
    motorVoltage.setDouble(rightShooterFalcon.getMotorOutputVoltage());
    leftMotorCurrent.setDouble(leftShooterFalcon.getSupplyCurrent());
    rightMotorCurrent.setDouble(rightShooterFalcon.getSupplyCurrent());
    targetSpeed.setDouble(rightShooterFalcon.getClosedLoopTarget() * ((10.0 / 2048.0) * 60));
    
    // update top shooter parameters
    topShooterSpeed.setDouble(topShooterFalcon.getSelectedSensorVelocity() * ((10.0 / 2048.0) * 60));
    topShootertargetSpeed.setDouble(topShooterFalcon.getClosedLoopTarget() * ((10.0 / 2048.0) * 60));
    
    // update hood parameters
    hoodTargetPos.setDouble(m_HoodTargetPos);
    hoodEstimatedPos.setDouble(m_HoodEstimatedPos);
  }
}

// enable

// 19:42
// F=0.0477
// P=0.7
// I=0.001
// Ilimit = 80,000

// 19:51 // near perfect if we get voltage compenstaiton working
// F=0.0477
// P=0.7
// I=0.001
// D=0.05
// Ilimit = 120,000

// 20:41 // reduced back down due to new battery (higher voltage)
// if possible, get
// F=0.0477
// P=0.4
// I=0.001
// D=0.05
// Ilimit = 120,000

// 21:03 // appears ok. maybe add ~100ms before first ball shoots to allow
// settling
// F=0.0477
// P=0.38
// I=0.0001
// D=0.05
// Ilimit = 120,000

// works. not sure if better than previous, but appears semi-ok
// P=0.45
// D=0.1
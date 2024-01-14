// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;

public class BallTargeting extends SubsystemBase {
  private Limelight m_ballCamera;
  private CameraTilt m_ballCameraTilt;

  private double m_notargettimer;

  // PID gains for rotating robot towards ball target
  private double kp = 0.20;
  private double ki = 0.0;
  private double kd = 0.0;
  
  // camera tilt controller
  private PIDController pidController = new PIDController(kp, ki, kd);

  private boolean IsTargettingBall;

  /** Creates a new BallTargeting. */
  public BallTargeting() {
    // create a camera
    m_ballCamera = new Limelight("limelight-ball");
    
    // create the camera tilting
    m_ballCameraTilt = new CameraTilt();

    // by default, set to look for balls
    SettoBallTarget();

    m_notargettimer=0.0;
    pidController.reset();
  }

// ------- SET and GET target types

/** Set to Ball Pipeline*/
public void SettoBallTarget()
{
  IsTargettingBall = true;

  // set ball pickup pipeline
  if (DriverStation.getAlliance() == Alliance.Red)
    m_ballCamera.setPipeline(1);
  else
    m_ballCamera.setPipeline(2);
}
/** Set to Climb Target */
public void SettoClimbTarget()
{ 
  IsTargettingBall = false;
  
  // set ball pickup pipeline
  if (DriverStation.getAlliance() == Alliance.Red)
     m_ballCamera.setPipeline(7);
  else
    m_ballCamera.setPipeline(6);
}

/** Returns true if currently looking for balls */
//public boolean isOnBallTarget()
//  { int pipeline = m_ballCamera.getPipeline();  return (pipeline>=1 && pipeline<=4); }
/** Returns true if currently looking for climb target */
//public boolean isOnClimbTarget()
//  { return(m_ballCamera.getPipeline() == 6); }



// -----------  Generic Target Functions -----

/** is target present? */
public boolean isTarget(){
    return m_ballCamera.isTargetPresent();
  }

/** finds the x-angle/rotation of target (deg)*/
public double getTargetHorAngle() {
  double Angle = m_ballCamera.getHorizontalTargetOffsetAngle();
  return Angle;
}

/** Returns vertical angle of target (deg)*/
public double getTargetVertAngle() {
  return m_ballCamera.getVerticalTargetOffsetAngle();
}



// --------- 
  /**Is there a ball (boolean)
   * Decides whether there is a ball based on area
   * Note: Area is represented as percentage of limelight screen
   * Returns boolean true if a ball is recognized */
  public boolean IsBall() {
    double ballArea = m_ballCamera.getTargetArea();
    return ((ballArea >= 0.1) && (m_ballCamera.getPipeline() !=6));
  }

  
  // if (!RobotContainer.ballTargeting.climbPipeline()){
  @Override
  public void periodic() {
  
    double adjust = 0.0;

    if (!IsTargettingBall)
    {
      m_ballCameraTilt.setAngle (-14.4);
    }
    else 
    {
      // we are in ball mode
      if (IsBall())
      {
        adjust = -pidController.calculate(getTargetVertAngle());
        if (Math.abs(adjust) <= 0.5)
         adjust = 0.0;
        m_ballCameraTilt.setAngle(m_ballCameraTilt.getAngle() + adjust);  
        m_notargettimer = 0.0;

        // if we are 'looking down', widen ball detection to look for non-square objects
        if (m_ballCameraTilt.getAngle() < -50.0)
        {
          if (DriverStation.getAlliance() == Alliance.Red)
           m_ballCamera.setPipeline(3);
         else
            m_ballCamera.setPipeline(4);
        }
        else
        {
         if (DriverStation.getAlliance() == Alliance.Red)
           m_ballCamera.setPipeline(1);
          else
           m_ballCamera.setPipeline(2);
        }
      }
    
      // we have no target. Increment timer
      else
      {
        m_notargettimer += 0.02;
        pidController.reset();
      }
    
      // we have no target - raise camera
      // reset pipeline to ordinary ball detection
      if (m_notargettimer > 1.0)
      {
        m_ballCameraTilt.setAngle (-14.4);
        if (DriverStation.getAlliance() == Alliance.Red)
          m_ballCamera.setPipeline(1);
        else
         m_ballCamera.setPipeline(2);
      }
    }
  }
 
}

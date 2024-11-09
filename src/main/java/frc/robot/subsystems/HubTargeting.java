// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HubTargeting extends SubsystemBase {
  private Limelight m_hubCamera;

  private GenericEntry m_ty;
  private GenericEntry m_distance;
  private GenericEntry m_RPM;
  private GenericEntry m_Hood;

  private GenericEntry m_targetWithinRange;
  private GenericEntry m_readyToShoot;
  private GenericEntry m_targetDetected;
  private GenericEntry m_estimatedHighBarTarget;
  private GenericEntry m_estimatedLowBarTarget;

  private GenericEntry m_RPMAdjust;
  public GenericEntry m_DistanceAdjust;
  private GenericEntry m_HoodAdjust;
  private GenericEntry m_TopRPMAdjust;

  public double m_OnTheFlyRPMAdjust = 0;

  // default shooter idle speed (rpm)
  private final double m_ShooterIdleSpeed = 1650.0;
  private final double m_TopShooterIdleSpeed = 2200.0;

  /** Creates a new HubTargeting. */
  public HubTargeting() {
    m_hubCamera = new Limelight("limelight-hub");
    SettoHubTarget();
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // update shuffleboard
    if (isTargetPresent()) {
      //RobotContainer.m_shooter.setShooterAngle(GetTargetHoodSetting());
      // RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.GetTargetRPM());
      // RobotContainer.m_shooter.setTopShooterSpeed(RobotContainer.hubTargeting.GetTopTargetRPM());
    }
    updateShuffleboard();
  }

  // ------- SET and GET target types

  /** Set to Ball Pipeline*/
  public void SettoHubTarget()
  { m_hubCamera.setPipeline(0); }


  /** Function to tell if target is visible in limelight.
   * @return boolean (true if target is in sight) */
  public boolean isTargetPresent(){
    return m_hubCamera.isTargetPresent();
  }

  /**
   * Decides if a target is present and in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    boolean target = m_hubCamera.isTargetPresent();
    double distance = EstimateDistance();

    // we have valid target if distance is >2.9m
    return (target == true && distance >= 2.90);
  }

  public double getBarTY() {
    return (m_hubCamera.getVerticalTargetOffsetAngle());
  }

  public boolean barReady() {
    // check if the bar is within range to climb
    double ty = m_hubCamera.getVerticalTargetOffsetAngle();
    System.out.println(ty);
    return (ty > -7 && ty != 0);
  }

  public boolean isBar() {
    // check if we see a bar
    System.out.println(m_hubCamera.isTargetPresent());
    return m_hubCamera.isTargetPresent();
  }

  public double estimateHorizontalDistanceToLowerBar() {
    double height = 0.153;
    double ty = m_hubCamera.getVerticalTargetOffsetAngle();
    double hypotenuse = 34.344*Math.pow(ty,2)-78.941*ty+28.857;
    return (Math.sqrt((Math.pow(hypotenuse, 2) - Math.pow(height, 2))));
  }

  public double estimateHorizontalDistanceToUpperBar() {
    double height = 0.495;
    double ty = m_hubCamera.getVerticalTargetOffsetAngle();
    double hypotenuse = 39.413*Math.pow(ty,2)-116.73*ty+85.611;
    return (Math.sqrt((Math.pow(hypotenuse, 2) - Math.pow(height, 2))));
  }

  /**
   * Estimates the distance of the hub
   * Note: Equation will change with mounted limelight and testing
   * 
   * @return distance in meters
   */
  public double EstimateDistance() {
    double ty = m_hubCamera.getVerticalTargetOffsetAngle();
    double distance;

    // based on test data from April 2 2022
    if (ty < 0.67)
      distance = 0.00016 * ty * ty * ty * ty + 0.000877 * ty * ty * ty + 0.008232 * ty * ty - 0.194295 * ty + 4.176152;
    else
      // re-used from last characterization
      distance = 0.0039 * ty * ty - 0.1523 * ty + 4.094;

    // add in adustment (if any) from shuffleboard
    distance += (m_DistanceAdjust.getDouble(0.0));

    // return distance estimate (m)
    return distance;
  }

  /**
   * finds angle of rotation to hub
   * 
   * @return rotation angle
   */
  public double getHubAngle() {
    double tx = m_hubCamera.getHorizontalTargetOffsetAngle();
    return tx;
  }

  /**
   * Finds if targeting is ready
   * Note: angle displacement parameters are placement holders
   * 
   * @return boolean
   */
  public boolean ReadyToShoot() {
    double targetAngle = getHubAngle();
    Boolean target = IsTarget();
    return (targetAngle <= 1.0 && targetAngle >= -1.0 && target == true);
  }

  /**
   * Finds the RPM (rates per minute) needed to shoot a distance
   * Note: equation will change with finished shooter + testing
   * @return RPM as double*/
  public double GetTargetRPM(double distance) {
    // based on test data from April 2 2022
    double m = distance;
    double RPM;
    if (m <= 6.50)
      RPM = 79.754607 * m * m * m - 1048.135892 * m * m + 4676.800317 * m - 5344.739444;
    else
      RPM = -155.138546*m*m +2579.107342*m - 7666.820108;  

    // from field testing April 7 2022
    if (m>6.30 && m<6.69)
      RPM += 30.0;
    if (m>4.10 && m<4.40)
      RPM -= 30.0;

    // April 9 2022 
    if (m<3.75)
      RPM += 50.0;

    // add in RPM adustment (if any) from shuffleboard
    RPM += m_RPMAdjust.getDouble(0.0);

    //Add in operator shot evaluation adjustment
    RPM += m_OnTheFlyRPMAdjust;

    return RPM;
  }

   public double GetTargetRPM()
   { return GetTargetRPM(EstimateDistance()); }

  
  
   public double GetTopTargetRPM(double distance) {
    // get distance in m
    double m = distance;
    
    // based on test data from April 2 2022
    double RPM;
    if (m < 5.50)
      RPM = 24.836680 * m * m + 21.019404 * m + 2409.906201;
    else
      RPM = 436.415649*m*m -6400.682739*m +25605.030690;
    
    // April 9 2022 
    if (m<3.75)
      RPM -= 50.0;
    
    /*if (m<3.75)
      RPM += 50.0;
    if (m<3.50)
      RPM += 50.0;
    if (m<3.25)
      RPM += 50.0; */

    // adjust top flywheel RPM
    RPM += m_TopRPMAdjust.getDouble(0.0);

    return RPM;
  }

  public double GetTopTargetRPM()
    { return GetTopTargetRPM(EstimateDistance()); }
   

  /** Finds the Shooter Hood actuator Setting need to shoot a distance
   * @return hood actuator setting*/
  public double GetTargetHoodSetting(double distance) {
    // get distance in m
    double m = distance;
    
    // based on test data from April 2 2022
    double hood;
    if (m <= 4.78)
      hood = -1.0;
    else
      hood = 0.018890 * m * m * m - 0.462281 * m * m + 3.744794 * m - 10.400234;

    // add in hood adustment (if any) from shuffleboard
    hood += m_HoodAdjust.getDouble(0.0);

    return hood;
  }
  
  public double GetTargetHoodSetting()
    { return GetTargetHoodSetting(EstimateDistance()); }
    

  /** Returns shooter idle speed (rpm) */
  public double getShooterIdleSpeed() {
    return m_ShooterIdleSpeed;
  }

  /** Returns top shooter idle speed (rpm) */
  public double getTopShooterIdleSpeed() {
    return m_TopShooterIdleSpeed;
  }

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Hub Target");

    // create controls to display limelight ty, target ready, estimated distance and
    // estimated RPM
    ShuffleboardLayout l1 = Tab.getLayout("Hub Target", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(2, 3);
    m_ty = l1.add("limelight ty", 0.0).getEntry();
    m_distance = l1.add("Estimated Target Distance", 0.0).getEntry();
    m_RPM = l1.add("Target RPM", 0.0).getEntry();
    m_Hood = l1.add("Target Hood", 0.0).getEntry();

    m_estimatedHighBarTarget = l1.add("high bar estimated distance", 0.0).getEntry();
    m_estimatedLowBarTarget = l1.add("low bar estimated distance", 0.0).getEntry();

    // does camera detect target
    m_targetDetected = Tab.add("Detected", false)
        .withPosition(2, 0).getEntry();
    m_targetWithinRange = Tab.add("Within Range", false)
        .withPosition(2, 1).getEntry();
    m_readyToShoot = Tab.add("Ready to Shoot", false)
        .withPosition(2, 2).getEntry();

    m_RPMAdjust = Tab.addPersistent("Shooter Speed Adjust (rpm)", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -200.0, "max", 300.0))
        .withPosition(3, 0)
        .withSize(3, 1)
        .getEntry();

    m_DistanceAdjust = Tab.addPersistent("Hub Distance Adjust (m)", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .withPosition(3, 1)
        .withSize(3, 1)
        .getEntry();

    m_HoodAdjust = Tab.addPersistent("Hood Adjust (m)", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -0.15, "max", 0.15))
        .withPosition(3, 2)
        .withSize(3, 1)
        .getEntry();

    m_TopRPMAdjust = Tab.add("Top Flywheel Speed Adjust (rpm)", 0.0)
                  .withWidget(BuiltInWidgets.kNumberSlider)
                  .withProperties(Map.of("min", -500.0, "max", 500.0))
                  .withPosition(3,3)
                  .withSize(3,1)
                  .getEntry();
  }


  /** Update subsystem shuffle board page with current targeting values */
  private void updateShuffleboard() {
    // target within range and ready to shoot
    m_targetDetected.setBoolean(m_hubCamera.isTargetPresent());
    m_targetWithinRange.setBoolean(IsTarget());
    m_readyToShoot.setBoolean(ReadyToShoot());
    m_estimatedHighBarTarget.setDouble(estimateHorizontalDistanceToUpperBar());
    m_estimatedLowBarTarget.setDouble(estimateHorizontalDistanceToLowerBar());

    // estimated target attributes
    m_ty.setDouble(m_hubCamera.getVerticalTargetOffsetAngle());
    m_distance.setDouble(EstimateDistance());
    m_RPM.setDouble(GetTargetRPM());
    m_Hood.setDouble(GetTargetHoodSetting());
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Lily was here 

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ReleaseBall;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  // boolean flag to indicate if robot has been initialized
  boolean robotIsInitialized = false;
  
  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.Init();
    // Initialise our RobotContainer. This will perform all our button bindings, and
    // put our
    // autonomous chooser on the dashboard.
    RobotContainer.init();

    // set targeting to ball
    RobotContainer.ballTargeting.SettoBallTarget();

    // reset the gyro when robot powered up
    RobotContainer.gyro.resetGyro();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("X In", OI.driverController.getLeftX());
    SmartDashboard.putNumber("Y In", OI.driverController.getLeftY());
    
    SmartDashboard.putBoolean("Button1", OI.driverController.getRawButton(1));
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
   
    // initialize robot subsystems    
    // RobotContainer.gyro.resetGyro(); // Todo: Removed as auto may start at variying angles
    RobotContainer.odometry.InitializefromShuffleboard();
    
    // set targeting to ball
    RobotContainer.ballTargeting.SettoBallTarget();
    RobotContainer.hubTargeting.SettoHubTarget();

    // robot is now initialized
    robotIsInitialized = true;
    autonomousCommand = RobotContainer.getAutonomousCommand();
    
    // schedule the autonomous command
    if (autonomousCommand != null)
      autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
   
    // if robot has not previously been initialized, then go ahead and initialize
    //if (robotIsInitialized == false) {
      // TODO: Removed for competition
      // RobotContainer.gyro.resetGyro();
      // RobotContainer.odometry.InitializetoZero();
      
      // robot is now initialized
      //robotIsInitialized = true;
    //}
   
   
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.getShooterIdleSpeed());
    RobotContainer.m_shooter.setTopShooterSpeed(RobotContainer.hubTargeting.getTopShooterIdleSpeed());

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // set targeting to ball
    RobotContainer.ballTargeting.SettoBallTarget();
    RobotContainer.hubTargeting.SettoHubTarget();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.gyro.resetGyro();
  }

  /** This function is called 
  periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TurnRobot extends Command {
  
  double m_angle;
  boolean m_relative;
  double m_endangle;
  double m_timeout;

  // speed to rotate robot - determined by PID controller
  double m_rotatespeed;
  double m_angleerror;
  double m_time;
  
  boolean m_cameraControlled;

  // PID gains for rotating robot towards ball target
  double kp = 0.0125;
  double ki = 0.0;
  double kd = 0.00125;
  PIDController pidController = new PIDController(kp, ki, kd);

  private Drivetrain m_drivetrain = RobotContainer.drivetrain;

  /** Turn to/by angle
   * Input: angle - degrees (-180<angle<180)
   * relative - true if relative to current angle, false if absolute to field
   * clockwise - true if rotate clockwise, false if counter-clockwise */
  public TurnRobot(double angle, boolean relative, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_relative = relative;
    m_timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // our current facing angle
    double ourcurrentangle = RobotContainer.gyro.continuousYaw();

    // set our ending angle based on whether angle is relative or absolute
    if (m_relative)
      m_endangle = ourcurrentangle + m_angle;
    else
      m_endangle = m_angle;

      // reset PID controller
      pidController.reset();

      m_angleerror = 0.0;
      m_time = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // increment time in command 
    m_time += 0.02;
    
    // determine speed to rotate robot
    if (m_relative)
      m_angleerror = m_endangle - RobotContainer.gyro.continuousYaw();
    else
      m_angleerror = m_endangle - RobotContainer.gyro.getYaw();
    
    // execute PID controller
    m_rotatespeed = pidController.calculate(m_angleerror);
    
    if (m_rotatespeed > 0.5)
      m_rotatespeed = 0.5;
    if (m_rotatespeed < -0.5)
      m_rotatespeed = -0.5;

    // rotate robot
    RobotContainer.drivetrain.drive(
      new Translation2d(0.0, 0.0), -m_rotatespeed * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop robot
    RobotContainer.drivetrain.drive(new Translation2d(0.0,0.0), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      // we are finished when within 2deg of target, or have timeed out
    return (Math.abs(m_angleerror) <=2.0 || m_time >=m_timeout);
  }
}

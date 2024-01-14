// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoDriveToPose extends Command {
  
  private Pose2d m_target;
  private double m_speed;
  private double m_rotspeed;
  private double m_timeout;
  private boolean m_recallPoint;

  private double m_time;

  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.12;
  private final double m_angletolerance = 2.0;

  // x, y, rotation PID controllers
  private PIDController m_xController = new PIDController(0.50, 0, 0.035);
  private PIDController m_yController = new PIDController(0.50, 0, 0.035);
  private PIDController m_rotController = new PIDController(0.01, 0, 0.001);

  public static double AngleDifference( double angle1, double angle2 )
  {
      double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
      return diff < -180 ? diff + 360 : diff;
  }

  /** Creates a new AutoDriveToPoseCommand. 
   * Use this to have command to drive back to coordinate provided to it */
  public AutoDriveToPose(Pose2d target, double speed, double rotationalspeed, double timeout) {
    addRequirements(RobotContainer.drivetrain);
    m_target = target;
    m_speed = speed;
    m_rotspeed = rotationalspeed;
    m_timeout = timeout;
    m_recallPoint = false;
  }

  /** use this during teleop to go to pre-recorded position*/
  public AutoDriveToPose(double speed, double rotationalspeed)
  {
    m_speed = speed;
    m_rotspeed = rotationalspeed;
    m_recallPoint = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time = 0.0;

    // recall previously saved point and use it as our destination
    if (m_recallPoint)
      m_target = RobotContainer.odometry.RecallPoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d curr = RobotContainer.odometry.getPose2d();
    
    // increment time
    m_time += 0.02;

    // execute PIDs
    double xSpeed = m_xController.calculate(curr.getX()- m_target.getX() );
    double ySpeed = -m_yController.calculate(curr.getY() - m_target.getY() );
    
    double rotSpeed = m_rotController.calculate(-AngleDifference(curr.getRotation().getDegrees(),m_target.getRotation().getDegrees()));
    //double rotSpeed = m_rotController.calculate(curr.getRotation().getDegrees() - m_target.getRotation().getDegrees());

    // limit speeds to allowable
    if (xSpeed > m_speed)
      xSpeed = m_speed;
    if (xSpeed < -m_speed)
      xSpeed = -m_speed;
    if (ySpeed > m_speed)
      ySpeed = m_speed; 
    if (ySpeed < -m_speed)
      ySpeed = -m_speed;  
    if (rotSpeed >m_rotspeed)
      rotSpeed = m_rotspeed;
    if (rotSpeed < -m_rotspeed)
      rotSpeed = -m_rotspeed;

    // drive robot according to x,y,rot PID controller speeds
    RobotContainer.drivetrain.drive(new Translation2d(xSpeed*RobotContainer.drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                                      ySpeed*RobotContainer.drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                                    rotSpeed*RobotContainer.drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                    true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // we have finished path. Stop robot
    RobotContainer.drivetrain.drive(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d curr = RobotContainer.odometry.getPose2d();

    // we are finished if we are within erorr of target or command had timeed out
    return (((Math.abs(m_target.getX() - curr.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - curr.getY()) <  m_positiontolerance) &&
          (Math.abs(m_target.getRotation().getDegrees() - curr.getRotation().getDegrees()) < m_angletolerance)) ||
          (m_time >= m_timeout));
  }

  


}
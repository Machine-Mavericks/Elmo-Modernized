// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
public class ApproachBall extends Command {
  /** Creates a new ApproachBall. */
  
  private Drivetrain m_drivetrain = RobotContainer.drivetrain;
  private Gyro m_gyro = RobotContainer.gyro;

// get angle to target
double TargetAngle = 0;

   // TODO: set gains
   double kp = 0.0125;
   double ki = 0.0;
   double kd = 0.0; //0.00015;
 
   PIDController pidController = new PIDController(kp, ki, kd);
  
  
  public ApproachBall() {
    // Use addRequirements() here to declare subsystem dependencies.
  
    addRequirements(m_drivetrain);
    addRequirements(m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double angle = 0.0;
    double speed = 0.0;

// do we have a valid target?
if ((RobotContainer.ballTargeting.IsBall())){

  
  TargetAngle = RobotContainer.ballTargeting.getTargetHorAngle();
    
  // determine angle correction - uses PI controller
  angle = pidController.calculate(TargetAngle+5.0);
  if (angle > 1.0)
    angle = 1.0;
  if (angle < -1.0)
    angle = -1.0;

  if (m_gyro.getYaw() >-80)
    speed = 0.2;
  else
    speed = 0.2 + 0.10*(-90 - m_gyro.getYaw())/10.0;

if (speed < 0.0)
  speed = 0.0;

  }   // end if we have a valid target


    RobotContainer.drivetrain.drive(
      new Translation2d(speed * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          0 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
          angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.drivetrain.drive(
      new Translation2d(0.0 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          0 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
          0.0* Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angle = m_gyro.getYaw();
    
    return ((angle >=87 && angle <=93) ||
    (angle <-87 && angle >=-93));

  }
}

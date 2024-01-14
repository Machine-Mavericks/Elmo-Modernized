// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbDrive extends Command {
  
  double motorSpeed;

  double targetTy = 0.0;
  double m_speedlimit = 0.0;
  double kp= 0.04;
  double ki= 0.00;
  double kd= 0.0;
  double time_ontarget;

  PIDController pidController = new PIDController(kp,ki,kd);

  /** Creates a new ClimbCommand. */
  public ClimbDrive(double angle, double speedlimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    addRequirements(RobotContainer.ballTargeting);
    addRequirements(RobotContainer.drivetrain);
    
    pidController.setIntegratorRange(-0.15, 0.15);

    // save ball camera target angle
    targetTy = angle;
    m_speedlimit = speedlimit;
  }

  // Called when the command is initially scheduled.
  double err;
  @Override
  public void initialize() {
    pidController.reset();
    err = 0.0;
    time_ontarget = 0.0;
  }
  
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed;
    
    //if (RobotContainer.ballTargeting.isTarget())
    if (Math.abs(RobotContainer.ballTargeting.getTargetVertAngle())>0.001)
    {  err = RobotContainer.ballTargeting.getTargetVertAngle() - targetTy;
      speed = -pidController.calculate(err);
    }
    else
      speed = 0.20;

    //limit our speed
    if (speed > m_speedlimit)
     speed = m_speedlimit;
    if (speed < -m_speedlimit)
      speed = -m_speedlimit;

    RobotContainer.drivetrain.drive(new Translation2d(speed*RobotContainer.drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                                      0.0), 0.0,true);


    if (Math.abs(RobotContainer.ballTargeting.getTargetVertAngle())>0.001 &&
    Math.abs(RobotContainer.ballTargeting.getTargetVertAngle() - targetTy) <=0.5)
    time_ontarget +=0.2;

    // if (DriverStation.getMatchTime() < 30.0) //do not deploy climber if it isn't endgame
    //   RobotContainer.climber.motorVelocity();
    // else
    // RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0.0,0.0), 0.0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are done if we find target and error less than 0.5deg
    return (time_ontarget > 0.2);
    //return Math.abs(RobotContainer.ballTargeting.getTargetVertAngle())>0.001 &&
    //Math.abs(RobotContainer.ballTargeting.getTargetVertAngle() - targetTy) <=0.3;
    
    //return (RobotContainer.ballTargeting.isTarget() && 
    //      (Math.abs(RobotContainer.ballTargeting.getTargetVertAngle() - targetTy) <=0.5));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;

public class SteerTowardsBall extends Command {

  // subsystems we are interfacing with
  private Drivetrain m_drivetrain = RobotContainer.drivetrain;
  private Gyro m_gyro = RobotContainer.gyro;
  private Intake m_intake = RobotContainer.intake;

  // angle to target
  double TargetAngle = 0;

  // PID gains for rotating robot towards ball target
  double kp = 0.0125;
  double ki = 0.0;
  double kd = 0.0;
  PIDController pidController = new PIDController(kp, ki, kd);

  // is this command automated or semi-automated?
  boolean m_automated;

  // speed limit when automated (0<speed<1.0)
  double m_speedLimitAuto;

  // command timeout time
  double m_timeoutlimit;
  double m_time;

  /** Steers robot towards ball
   * Input: true if fully automated, false if only sem-automated */
  public SteerTowardsBall(boolean automated, double timeout) {
    
    // this command requires use of drivetrain and gyro
    addRequirements(m_drivetrain);
    addRequirements(m_gyro);
    
    m_automated = automated;
    m_timeoutlimit = timeout;

    // assume speed limit of 0.5
    // TODO ken add comment
    m_speedLimitAuto= 0.65;
  }
  
  /** Steers robot towards ball
   * Input: true if fully automated, false if only sem-automated,
   *        speed limit (0 to 1.0) if automated */
  public SteerTowardsBall(boolean automated, double timeout, double speedlimit) {
    // this command requires use of drivetrain and gyro
    addRequirements(m_drivetrain);
    addRequirements(m_gyro);

    m_automated = automated;
    m_timeoutlimit = timeout;
    m_speedLimitAuto= speedlimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    RobotContainer.intake.setMotorSpeed(0.5);
    // set initial time
    m_time = 0.0;

    // reset pid controller
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if automated, assume 50% speed, in manual get speed from joystick
    double xInput;
    if (m_automated)
      xInput = m_speedLimitAuto;
    else
      xInput = m_speedLimitAuto*0.6; // ball pickup is completely autonomous
      //xInput = Math.sqrt(Math.pow(OI.driverController.getLeftY(), 2) + Math.pow(OI.driverController.getLeftX(), 2));
    
    // assume sideway speed of 0% unless determined otherwise
    double yInput = 0.0;
    
    // assume rotation not needed unless proven otherwise
    double rotate = 0.0;

    // increase out time in command
    m_time += 0.02;

    // do we have a valid target?
    if ((RobotContainer.ballTargeting.IsBall())){

      TargetAngle = RobotContainer.ballTargeting.getTargetHorAngle();
    
      // determine angle correction - uses PI controller
      // limit rotation to +/- 100% of available speed
      rotate = pidController.calculate(TargetAngle);
      if (rotate > 1.0)
        rotate = 1.0;
      if (rotate < -1.0)
        rotate = -1.0;

      // if not fully automatic, get joystick inputs
      if (m_automated)
      {
        // slow down forward speed if large angle to allow robot to turn
        // at 25deg,  speed = 0.5 - 0.004(25)) = 0.5 - 0.1) = 0.4
        xInput = m_speedLimitAuto; //- 0.004*m_speedLimitAuto* Math.min(25.0, Math.abs(TargetAngle));
        //xInput = OI.driverController.getLeftY();
        //if (xInput<0.0)
        //  xInput=0.0;
      }
      

    }   // end if we have a valid target
    
  // command robot to drive - using robot-relative coordinates
  RobotContainer.drivetrain.drive(
    new Translation2d(xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
        rotate * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are finished if max time in our command expires
    // automatically finish command if we already sense having two balls
    return ((m_automated && (m_intake.GetIntakeLimitSwitchStatus() || (m_time >= m_timeoutlimit)))
          || ( RobotContainer.intake.GetIntakeLimitSwitchStatus() && (!RobotContainer.lifter.liftLimit.get()))
          );

  }
}

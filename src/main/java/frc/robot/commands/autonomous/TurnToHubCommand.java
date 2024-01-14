// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/**
 * Command which turns the robot at a provided speed until the hub has been detected
 */
public class TurnToHubCommand extends Command {
  double turnSpeed;
  long timeout;
  double err;
  /** 
   * Creates a new TurnToHubCommand. 
   * @param turnSpeed Speed to make turn, in %{@link Drivetrain#MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND}
   * @param timeout Timeout in seconds
  */
  public TurnToHubCommand(double turnSpeed, double timeout) {
    addRequirements(RobotContainer.drivetrain);
    this.turnSpeed = turnSpeed;
    this.timeout = (int) (timeout*1000);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialise timeout
    this.timeout += System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Determine direction from odometry
    Pose2d pose = RobotContainer.odometry.getPose2d();
    Pose2d hub = new Pose2d(6.25, 3.125, new Rotation2d(0));
    Pose2d hubDir = pose.relativeTo(hub);
    
    double targetAngle = Math.toDegrees(Math.atan2(hubDir.getY(), hubDir.getX()));

    err = (-pose.getRotation().getDegrees() + targetAngle) % 360;
    if(Math.abs(err) > 180) err += 360;
    err %= 360;
    if(err > 180) err = err-360;

    turnSpeed = Math.abs(turnSpeed);
    if(err < 0){
      turnSpeed = -turnSpeed;
    }
    // Start driving desired direction
    RobotContainer.drivetrain.drive(new Translation2d(0,0), turnSpeed*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when within 5 degrees of facing the hub
    return ((RobotContainer.hubTargeting.isTargetPresent() &&(Math.abs(RobotContainer.hubTargeting.getHubAngle()) <4.0))
    || System.currentTimeMillis() > timeout);

  }
}

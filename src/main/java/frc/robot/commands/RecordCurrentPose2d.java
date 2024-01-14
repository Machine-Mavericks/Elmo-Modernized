// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** Command records current Pose2D position - obtained by Odometry
 * After that, command immediately finishes */
public class RecordCurrentPose2d extends Command {
  /** Creates a new RecordCurrentPose2d. */
  public RecordCurrentPose2d() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.odometry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.odometry.RecordPose2d(RobotContainer.odometry.getPose2d(),0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ReleaseBall extends Command {

  /** Creates a new ReleaseBall. */
  public ReleaseBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.lifter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the intake and the lifter
    RobotContainer.intake.setMotorSpeed(-Intake.MOTORSPEED);
    RobotContainer.lifter.releaseBalls();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.lifter.releaseBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // stop the intake and lifter
    RobotContainer.intake.setMotorSpeed(0);
    RobotContainer.lifter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

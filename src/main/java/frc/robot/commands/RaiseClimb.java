// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RaiseClimb extends Command {
  //private double m_encoderi;
  private int m_encDisp;

  /** Creates a new RaiseClimb. */
  public RaiseClimb(int encDisp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    m_encDisp = encDisp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set base encoder value
    //m_encoderi = RobotContainer.climber.encoderVal();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
    RobotContainer.climber.motorVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (RobotContainer.climber.encoderVal() > (m_encoderi + m_encDisp));
    return RobotContainer.climber.encoderVal() > m_encDisp;
  }
}

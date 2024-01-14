// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  private int m_timer = 0;
  // NOTE: Currently unsed, may be reimplemented at later date?
  // time it will take until the intake retracts, set at 5 seconds currently
  private static final int END_TIME_TICKS = 5 * 50;

  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.lifter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the intake and the lifter
    RobotContainer.intake.setMotorSpeed(Intake.MOTORSPEED);
    //check if limit switch is activated
    if (!RobotContainer.lifter.liftLimit.get()){
      RobotContainer.lifter.stopMotor();
    }
    else{
      RobotContainer.lifter.liftBalls();
    }
    // initialize timer at start of command
    m_timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // increment timer
    m_timer++;

    //check if limit switch is activated
    if (!RobotContainer.lifter.liftLimit.get()){
      RobotContainer.lifter.stopMotor();
    }
    else{
      RobotContainer.lifter.liftBalls();
    }
    
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
    return ((m_timer >= END_TIME_TICKS) || ( RobotContainer.intake.GetIntakeLimitSwitchStatus() && (!RobotContainer.lifter.liftLimit.get())));
  }
}

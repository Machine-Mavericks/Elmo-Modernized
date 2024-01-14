// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

/** Command used to delay start of autonomous command
 *  Place command as first step of autonomous routine
 *  Reads delay time from shuffleboard page */
public class AutoDelayCommand extends Command {
  
  private Timer m_Timer;
  private double m_delayTime;
  
  /** Creates a new AutoDelayCommand. */
  public AutoDelayCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer = new Timer();
    m_Timer.reset();
    m_Timer.start();
    
    // get time to delay from shuffleboard OI
    m_delayTime = RobotContainer.shuffleboard.getAutoDelay();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.hasElapsed(m_delayTime);
  }
}

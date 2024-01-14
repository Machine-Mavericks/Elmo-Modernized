// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbGroup extends SequentialCommandGroup {
  /** Creates a new ClimbGroup. */
  public ClimbGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  addCommands(
    new ClimbStart(),
    // switch ball camera to look for line
    new InstantCommand(()-> RobotContainer.ballTargeting.SettoClimbTarget()), 
  
    // move robot until angle in camera is 0.0deg
    new ClimbDrive(2.5, 0.2),

    //new DelayCommand(5.0),
    // raise climber
    new RaiseClimb(300000),

    new ClimbDrive(-6.20, 0.10),

    // lower climber
    new RaiseClimb(400000)
    //new DelayCommand(30.0)
  );
  
    /*@Override
    public boolean isFinished(){
      return super.isFinished() && true;
    } */

  }
}

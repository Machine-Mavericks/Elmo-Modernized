// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AnywhereTwoBallAuto extends SequentialCommandGroup {
  /** Creates a new AnywhereTwoBallAuto. */
  public AnywhereTwoBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // ramp up shooter speed in preparation to shoot balls  
      new InstantCommand(()-> RobotContainer.m_shooter.setShooterSpeed(1600.0)),  
      new InstantCommand(()-> RobotContainer.m_shooter.setTopShooterSpeed(2800.0)),
      
      // auto delay per shuffleboard value
      new AutoDelayCommand(),
      // Intake the ball
      new SteerTowardsBall(true, 2.5, 0.2),
      // Shoot all balls
      new AutoShootAllCommand(false).deadlineWith(new SteerTowardsHub())
      
    );
  }
}

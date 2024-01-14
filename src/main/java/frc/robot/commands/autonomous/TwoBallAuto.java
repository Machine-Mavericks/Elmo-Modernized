// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;

/**
 * Basic auto which starts on right tarmac, grabs nearest ball, and shoots both
 */
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public TwoBallAuto() {
    addCommands(
      // ramp up shooter speed in preparation to shoot balls  
      new InstantCommand(()-> RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.getShooterIdleSpeed())),
      // auto delay per shuffleboard value
      new AutoDelayCommand(),
      // Intake the ball
      new SteerTowardsBall(true, 2.0),
      // // Turn left until hub is in view
      new TurnToHubCommand(0.2, 1),
      // // Shoot first ball
      new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub()),
      // // Shoot second ball
      new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub())
    );
  }
}

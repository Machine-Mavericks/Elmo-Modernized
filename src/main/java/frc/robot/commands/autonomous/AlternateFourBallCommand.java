// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveToPose;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlternateFourBallCommand extends SequentialCommandGroup {
  /** Creates a new AlternateFourBallCommand. */
  public AlternateFourBallCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // pickup side ball and shoot both balls
      new AnywhereTwoBallAuto(),
      // ramp up shooter speed in preparation to shoot balls  
      new InstantCommand(()-> RobotContainer.m_shooter.setShooterSpeed(4450.0)),  
      // Intake the ball
      //new SteerTowardsBall(true, 4.0, 0.2),
      new AutoDriveToPose(new Pose2d(new Translation2d(-4.0, -1.0), Rotation2d.fromDegrees(-90.0)), 0.7, 0.10, 15.0),
      // Drive back a bit
      new AutoDriveToPose(new Pose2d(new Translation2d(-6.0, 0.5), Rotation2d.fromDegrees(-128.0)), 0.7, 0.10, 15.0),
      new SteerTowardsBall(true, 2.0, 0.3),
      // wait for 300ms to try and get human ball. after 300ms, start next command
      new SteerTowardsBall(true, 0.4, 0),
      // switch on intake to allow human ball player to provide ball
      new InstantCommand(()-> RobotContainer.intake.setMotorSpeed(0.5)),
      new AutoDriveToPose(new Pose2d(new Translation2d(-4.95, -0.42), Rotation2d.fromDegrees(-150.0)), 0.7, 0.10, 15.0),
      // Shoot first ball
      //new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub()),
      // Shoot first ball
      //new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub())
      new AutoShootAllCommand(false).deadlineWith(new SteerTowardsHub()),

      // switch on intake to allow human ball player to provide ball
      new InstantCommand(()-> RobotContainer.intake.setMotorSpeed(0))
    );
  }
}

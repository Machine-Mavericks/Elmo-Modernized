// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveToPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  public FiveBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // pickup side ball and shoot both balls
      new AnywhereTwoBallAuto(),
      // Drive back to shooting position
      //new AutoDriveToPose(new Pose2d(new Translation2d(0.35, -1.86), Rotation2d.fromDegrees(-171.0)), 0.2, 0.10, 10.0),      
      new AutoDriveToPose(new Pose2d(new Translation2d(-1.46, 0.74), Rotation2d.fromDegrees(-157)), 0.45, 0.20, 10.0),
      // was -1.70, 0.69, -147.2
      // Intake the ball
      new SteerTowardsBall(true, 2.0, 0.3),
      // Shoot first ball
      new AutoShootAllCommand(false).deadlineWith(new SteerTowardsHub()),
      //new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub()),
      // Drive to back corner
      //new AutoDriveToPose(new Pose2d(new Translation2d(-5.78, 0.134), Rotation2d.fromDegrees(-147.9)), 0.45, 0.15, 15.0),
      // ramp up shooter speed in preparation to shoot balls  
      new InstantCommand(()-> RobotContainer.m_shooter.setShooterSpeed(2500.0)), 
      new InstantCommand(()-> RobotContainer.m_shooter.setTopShooterSpeed(2200.0)),
      // Intake the ball
      //new SteerTowardsBall(true, 4.0, 0.2),
      // Drive back a bit
      new AutoDriveToPose(new Pose2d(new Translation2d(-5.48, -0.03), Rotation2d.fromDegrees(-145.0)), 0.7, 0.10, 15.0),
      // -6.12, -0.165, -158deeg
      // -5.48, -0.03, -145deg
      
      new SteerTowardsBall(true, 2.0, 0.3),

      // wait for 300ms to try and get human ball. after 300ms, start next command
      new SteerTowardsBall(true, 0.8, 0),
      
      // switch on intake to allow human ball player to provide ball
      new InstantCommand(()-> RobotContainer.intake.setMotorSpeed(0.5)),
      
      new InstantCommand(()-> RobotContainer.m_shooter.setShooterSpeed(1700.0)), 
      new InstantCommand(()-> RobotContainer.m_shooter.setTopShooterSpeed(2800.0)),

      new AutoDriveToPose(new Pose2d(new Translation2d(-3.02, -0.30), Rotation2d.fromDegrees(-146.7)), 0.7, 0.20, 10.0),
      //new AutoDriveToPose(new Pose2d(new Translation2d(-1.46, 0.74), Rotation2d.fromDegrees(-157)), 0.7, 0.20, 10.0),
      //new AutoDriveToPose(new Pose2d(new Translation2d(-4.95, -0.42), Rotation2d.fromDegrees(-150.0)), 0.7, 0.10, 15.0),
      
      // Shoot balls
      new AutoShootAllCommand(false).deadlineWith(new SteerTowardsHub()),

      // switch on intake to allow human ball player to provide ball
      new InstantCommand(()-> RobotContainer.intake.setMotorSpeed(0))
      );
  }

  @Override
  public boolean isFinished(){
    return super.isFinished() && true;
  }
}

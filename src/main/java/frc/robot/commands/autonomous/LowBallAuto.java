// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveToPose;
import frc.robot.commands.DelayCommand;

/**
 * Basic auto which starts on right tarmac, grabs nearest ball, and shoots both
 */
public class LowBallAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public LowBallAuto() {
    addCommands(
      new AutoShootCommand(AutoShootCommand.LOW_SPEED),
      new DelayCommand(5),
      // Todo: Set position
      new AutoDriveToPose(new Pose2d(new Translation2d(4, 4), new Rotation2d(0)), 0.5, 0.5, 3)
    );
  }
}

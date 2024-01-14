package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveToPose;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;

public class TerminalBallAutoCommand extends SequentialCommandGroup {
    public TerminalBallAutoCommand() {
        addCommands(
            // Make a dash for ball near station
            new AutoDriveToPose(new Pose2d(new Translation2d(1.5, 1.70), Rotation2d.fromDegrees(-135)), 0.75, 0.25, 30),
            new DelayCommand(0.25),
            new SteerTowardsBall(true, 2, 0.2),
            new DelayCommand(0.25),
            new SteerTowardsBall(true, 5, 0),
            // Drive back to shooting position
            new AutoDriveToPose(new Pose2d(new Translation2d(3, 2), Rotation2d.fromDegrees(-135)), 0.5, 0.10, 2),
            // Turn left until hub is in view
            new TurnToHubCommand(0.20, 2),
            // Shoot first ball
            new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub()),
            // Shoot second ball
            new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub())
        );
    }
}

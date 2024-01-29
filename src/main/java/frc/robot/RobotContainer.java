// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AimThenShoot;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ReleaseBall;
import frc.robot.commands.ShotEvaluationCommand;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.autonomous.AlternateFourBallCommand;
import frc.robot.commands.autonomous.AnywhereTwoBallAuto;
import frc.robot.commands.autonomous.FiveBallAuto;
import frc.robot.subsystems.BallTargeting;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.HubTargeting;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDBlinkin;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.PowerPanel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveOdometry;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final double updateDt = 0.02;

  // Create robot's shuffboard operator interface
  public static final ShuffleboardOI shuffleboard = new ShuffleboardOI();

  // The robot's subsystems are defined here...
  public static final Gyro gyro = new Gyro();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final SwerveOdometry odometry = new SwerveOdometry();
  //public static final PowerPanel panel = new PowerPanel(); // Todo: deal with it.
  public static final LEDBlinkin LEDStrip = new LEDBlinkin();
  public static final Shooter m_shooter = new Shooter();
  public static final Lifter lifter = new Lifter();
  public static final Intake intake = new Intake();
  public static final BallTargeting ballTargeting = new BallTargeting();
  public static final HubTargeting hubTargeting = new HubTargeting();
  public static final Climber climber = new Climber();

  /**
   * Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public static void init() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    LEDStrip.setDefaultCommand(new LEDCommand());

    // CameraServer.	startAutomaticCapture(0);
    // CameraServer.	startAutomaticCapture(1);
    // CameraServer.	startAutomaticCapture(2);
    // CameraServer.	startAutomaticCapture(3);
    // CameraServer.	startAutomaticCapture(4);
    // CameraServer.	startAutomaticCapture(5);
    // CameraServer.	startAutomaticCapture(6);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {

    OI.shootButton.whileTrue(new AimThenShoot());

    // TODO: Disable binding for competition use
    OI.zeroButton.whileTrue(new RunCommand(() -> gyro.resetGyro()));
    OI.intakeButton.whileTrue(new IntakeCommand());
    OI.ballTrackingButton.whileTrue(new SteerTowardsBall(false, 20.0));

    OI.overshootButton.whileTrue(new ShotEvaluationCommand(ShotEvaluationCommand.ShotType.Overshoot));
    OI.undershootButton.whileTrue(new ShotEvaluationCommand(ShotEvaluationCommand.ShotType.Undershoot));
    OI.shothitButton.whileTrue(new ShotEvaluationCommand(ShotEvaluationCommand.ShotType.Hit));
    OI.bounceoutButton.whileTrue(new ShotEvaluationCommand(ShotEvaluationCommand.ShotType.BouncedOut));
    
    OI.releaseBallButton.whileTrue(new ReleaseBall());

    //OI.climbButton.whileTrue(new ClimbGroup());
    //OI.manualClimbButton.whileTrue(new ClimbCommand());
 }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if (RobotContainer.shuffleboard.m_selectedPath == 0)
      return new AnywhereTwoBallAuto();
    else if (RobotContainer.shuffleboard.m_selectedPath == 1)
      return new FiveBallAuto();
    else if (RobotContainer.shuffleboard.m_selectedPath == 2)
      return new AlternateFourBallCommand();
    else
      return new AnywhereTwoBallAuto();
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {

  private Drivetrain m_drivetrain;

  private PIDController m_headingPID = new PIDController(0.01, 0, 0);
  // Use Double class so it can be set to null
  private Double m_PIDTarget = null;
  private long m_pidDelay = -1;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PIDTarget = null;
    m_pidDelay = 10; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Driver inputs, should be in range [-1,1]
    //SmartDashboard.putNumber("SDB", SlowDownButton);

    double xInput = OI.getXDriveInput();
    double yInput = OI.getYDriveInput();
    double rotInput = -OI.getRotDriveInput();

    // If no rotational input provided, use PID to hold heading
    // When the zero button is pressed force reset to prevent jumping
    if(rotInput == 0 && !OI.zeroButton.getAsBoolean()){
      if(m_pidDelay > 0) m_pidDelay --;
      else {
        // If the target is unset, set it to current heading
        if(m_PIDTarget == null){
          m_PIDTarget = RobotContainer.gyro.getYaw();
          m_headingPID.reset(); // Clear existing integral term as may accumulate while not in use
          //m_headingPID.setSetpoint(m_PIDTarget);
        }
        
        // if we are not moving robot, then apply 0 rotation speed to straighten wheels
        if (xInput==0.0 && yInput==0.0)
          rotInput = 0.0;
        else         // Compute rotational command from PID controller
          rotInput = m_headingPID.calculate(Utils.AngleDifference(m_PIDTarget, RobotContainer.gyro.getYaw()));
      }
    } else {
      // If there is input, set target to null so it's properly reset next time
      m_PIDTarget = null;
      m_pidDelay = 10; 
    }

    m_drivetrain.drive(new Translation2d(-yInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, xInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND), rotInput*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns false when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.logging.ShooterLog;
import frc.robot.logging.ShooterLog.ShotData;
import frc.robot.subsystems.HubTargeting;
import frc.robot.subsystems.Lifter;

public class ShotEvaluationCommand extends Command {

  public enum ShotType {Overshoot, Undershoot, Hit, BouncedOut};
  public static Pose2d RobotPose;

  private final double HubDistanceIncrement = 0.10;

  Lifter m_lifter = RobotContainer.lifter;
  HubTargeting m_hubTargeting = RobotContainer.hubTargeting;
  private ShotType m_shotType;

  //Logs odometry and shot successfullness, but does nothing with it, implement logging to file later.
  // private ArrayList<ArrayList<String>> ShotList = new ArrayList<ArrayList<String>>();

  /** Creates a new shotEvaluationCommand. */
  public ShotEvaluationCommand(ShotType shotType) {
    m_shotType = shotType;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotPose = RobotContainer.odometry.getPose2d();
    SmartDashboard.putNumber("array size",ShooterLog.instance().pastShots.size());

    if (m_lifter.shotsTaken > ShooterLog.instance().pastShots.size()){

    ShooterLog.instance().logShot(new ShooterLog.ShotData(m_shotType,RobotPose.getX(),RobotPose.getY(),RobotPose.getRotation().getDegrees()));

    double Offset;

    ArrayList<ShotData> pastShots = ShooterLog.instance().pastShots;

    if ((pastShots.size()>=2) && pastShots.get(pastShots.size()-1).type == pastShots.get(pastShots.size()-2).type){
      
      switch (m_shotType) {
        case BouncedOut:
          Offset = 0;
          break;
        case Hit:
          Offset = 0;
          break;
        case Undershoot:
          Offset = HubDistanceIncrement;
          break;
        case Overshoot:
          Offset = -HubDistanceIncrement;
          break;
        default:
          Offset = 0.0;
          break;
      }
      m_hubTargeting.m_DistanceAdjust.setDouble(m_hubTargeting.m_DistanceAdjust.getDouble(0.0)+ Offset);
    }
  }
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
    return true;
  }
}

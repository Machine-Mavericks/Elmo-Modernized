// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Swerve odometry is used to estimate current robot x, y position and angle.
// x and y coordinates are relative to when odometry was last reset

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveOdometry extends SubsystemBase {

  // constant to convert degrees to radians
  
  final float DEGtoRAD = (float) (3.1415926 / 180.0);

  // create swerve drive odometry object
  private SwerveDriveOdometry m_odometry;

  // subsystem shuffleboard controls
  private GenericEntry m_robotX;
  private GenericEntry m_robotY;
  private GenericEntry m_robotAngle;
  private GenericEntry m_gyroAngle;

  private GenericEntry m_initialX;
  private GenericEntry m_initialY;
  private GenericEntry m_initialAngle;

  /** Creates a new SwerveOdometry. */
  public SwerveOdometry() {

    // create robot odometry - set to (0,0,0)(x,y,ang)

    // initialize swerve drive odometry
    m_odometry = new SwerveDriveOdometry(RobotContainer.drivetrain.getKinematics(),
        new Rotation2d(0.0),
        RobotContainer.drivetrain.getSwervePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    // create odometry shuffleboard page
    initializeShuffleboard();
  }

  // -------------------- Initialize and Update Odometry Methods

  /**
   * Initialize robot odometry use shuffleboard settings and current gyro angle
   */
  public void InitializefromShuffleboard() {
    setPosition(m_initialX.getDouble(0.0),
        m_initialY.getDouble(0.0),
        m_initialAngle.getDouble(0.0),
        0.0);
  } // get gyro angle from subsystem

  /** Initialize robot odometry to zero */
  public void InitializetoZero() {
    setPosition(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Used to set or reset odometry to fixed position
   * x, y displacement in m, robot angle in deg, gyro in deg
   */
  public void setPosition(double x, double y, double robotangle, double gyroangle) {

    // make robot position vector
    Pose2d position = new Pose2d(x, y, new Rotation2d(robotangle * DEGtoRAD));

    // set robot odometry
    m_odometry.resetPosition(new Rotation2d(gyroangle * DEGtoRAD), RobotContainer.drivetrain.getSwervePositions(), position);  // new Rotation2d(gyroangle * DEGtoRAD)
  }

  public void setPositionHub(double dist) {
    double theta = getAngle()-180;
    double x = 8.23-dist*Math.cos(Math.toRadians(theta));
    double y = 4.11-dist*Math.sin(Math.toRadians(theta));
    setPosition(x,y,getAngle(),RobotContainer.gyro.getYaw());
  }

  /** Update current robot dometry - called by scheduler at 50Hz */
  @Override
  public void periodic() {

    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro.getYaw() * DEGtoRAD);

    // get positions of all swerve modules from subsystem
    SwerveModulePosition[] positions = RobotContainer.drivetrain.getSwervePositions();
    
    // ensure we have proper length array of positions before accessing elements of array
    if (positions.length >=4) {
      // update the robot's odometry
      m_odometry.update(gyroangle, positions);
    }
    
    // update odemetry shuffleboard page
    updateShuffleboard();
  }

  // -------------------- Robot Current Odometry Access Methods --------------------

  /** return robot's current position vector Pose2d */
  public Pose2d getPose2d() {
    return new Pose2d(getX(), getY(), new Rotation2d(getAngle() * DEGtoRAD));
  }

  /** Return current odometry x displacement (in m) */
  public double getX() {
    return m_odometry.getPoseMeters().getX();
  }

  /** Return current odometry y displacement (in m) */
  public double getY() {
    return m_odometry.getPoseMeters().getY();
  }

  // return current odometry angle (in deg)
  public double getAngle() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }


  // ----------------- FUnctions to record/recall Pos2d

  Pose2d m_MemPoints[] = {new Pose2d(0,0,new Rotation2d(0.0)),
                            new Pose2d(0,0,new Rotation2d(0.0)),
                            new Pose2d(0,0,new Rotation2d(0.0)) };

  /** saves Pose2D coordinate for later recall
   * num = 0 to 2 (three memories available) */
  public void RecordPose2d(Pose2d point, int num)
  {
      if (num<m_MemPoints.length)
        m_MemPoints[num] = point;
  }

  /** recalls Pose2D coordinate previously saved 
   * num = 0 to 2 (three memories available) */
  public Pose2d RecallPoint(int num)
  {
    // return saved point.  If not in range, simply return 0,0,0 point
    if (num<m_MemPoints.length)
      return m_MemPoints[num];
    else
      return new Pose2d(0,0,new Rotation2d(0.0));
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Odometry");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Odometry", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_robotX = l1.add("X (m)", 0.0).getEntry();
    m_robotY = l1.add("Y (m)", 0.0).getEntry();
    m_robotAngle = l1.add("Angle(deg)", 0.0).getEntry();
    m_gyroAngle = l1.add("Gyro(deg)", 0.0).getEntry();

    // Controls to set initial robot position and angle
    ShuffleboardLayout l2 = Tab.getLayout("Initial Position", BuiltInLayouts.kList);
    l2.withPosition(4, 0);
    l2.withSize(1, 3);
    m_initialX = l2.add("X (m)", 0.0).getEntry();           // eventually can use .addPersistent once code finalized
    m_initialY = l2.add("Y (m)", 0.0).getEntry();           // eventually can use .addPersentent once code finalized
    m_initialAngle = l2.add("Angle(deg)", 0.0).getEntry();  // eventually can use .addPersentent once code finalized
  }

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    // write current robot odometry
    m_robotX.setDouble(getX());
    m_robotY.setDouble(getY());
    m_robotAngle.setDouble(getAngle());
    m_gyroAngle.setDouble(getAngle());
  }

} // end SwerveOdometry Class

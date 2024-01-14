// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveOdometry;


/** Command to Follow Path */
public class FollowPath extends Command {

    private Drivetrain m_drivetrain = RobotContainer.drivetrain;
    private Trajectory trajectory;

    private Timer timer;
    private final SwerveOdometry m_odometry = RobotContainer.odometry;

    // PIDs gains for X and Y position controllers
    private double p = 5;
    private double i = 0;
    private double d = 0; 

    
    private ChassisSpeeds speeds;

    private HolonomicDriveController driveController;

    //private double m_endRobotAngle;
    private double m_RobotRotationRate;
    private double m_RobotAngle;

    // Measured in m/s and m/s/s
    private final double MAX_VELOCITY = 1.0;
    private final double MAX_ACCELERATION = 0.5;

    // Path parameters passed to us
    double [][] m_PathPoints;
    double m_PathStartAngle;
    double m_PathEndAngle;
    double m_PathStartSpeed;
    double m_PathEndSpeed;
    double m_PathEndRobotAngle;
    boolean m_PathReverse;
    boolean m_PathRotate;

    /** Follow Generic Path
     * Inputs:  points - coordinates that define path
     *          startAngle - starting angle of path
     *          endAngle - ending angle of path
     *          startSpeed - starting speed of robot (m/s)
     *          endSpeed - ending speed of robot (m/s)
     */
    public FollowPath(double[][] points, double startAngle, double endAngle, double startSpeed,
            double endSpeed, double endRobotAngle, boolean reverse, boolean rotatepath) {
        
        // store the path paremters provided to us
        m_PathPoints = points;
        m_PathStartAngle = startAngle;
        m_PathEndAngle = endAngle;
        m_PathStartSpeed = startSpeed;
        m_PathEndSpeed = endSpeed;
        m_PathEndRobotAngle = endRobotAngle;
        m_PathReverse = reverse;
        m_PathRotate = rotatepath;

        // set up timer to track time in the path
        timer = new Timer();

        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Create necessary profiled PID controller and configure it to be used with the
        // holonomic controller
        ProfiledPIDController rotationController = new ProfiledPIDController(
                2.5,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        // Create main holonomic drive controller
        // Note: From drive testing Feb10/2022, y axis in path controller is reverse from odometry
        // use negative y-axis controller gains to avoid positive feedback!
        driveController = new HolonomicDriveController(
                new PIDController(p, i, d), new PIDController(-p, -i, -d), rotationController);
        driveController.setEnabled(true);

        // go ahead and calculate our trajectory
        trajectory = calculateTrajectory(m_PathPoints,
                                        m_PathStartAngle,
                                        m_PathEndAngle,
                                        m_PathStartSpeed,
                                        m_PathEndSpeed,
                                        m_PathReverse,
                                        m_PathRotate);


        // beginning angle of robot - set to current angle
        m_RobotAngle = RobotContainer.gyro.getYaw();
        
        // determine determined robot rotation rate
        double rotatetime = trajectory.getTotalTimeSeconds();
        if (rotatetime!=0.0)
        {
            // if we are rotating path, then robot already 0deg relative to path start
            // otherwise, need to account for initial robot angle to determine how much additional to rotate
            if (!m_PathRotate)
                m_RobotRotationRate = (m_PathEndRobotAngle-m_RobotAngle)/rotatetime;
            else
                m_RobotRotationRate = m_PathEndRobotAngle/rotatetime;
        }
        else
            m_RobotRotationRate = 0.0;


        // Start timer when path begins
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get next trajectory state from our path
        State targetPathState = trajectory.sample(timer.get());

        // update our target robot rotation angle
        m_RobotAngle += m_RobotRotationRate*0.02;

        // set robot's angle
        Rotation2d desiredAngle = new Rotation2d(m_RobotAngle*3.1415/180.0);

        // get our current odeometry Pose
        Pose2d odometryPose = m_odometry.getPose2d();

        // determine robot chassis speeds
        speeds = driveController.calculate(odometryPose, targetPathState, desiredAngle);
        
        // instruct drive system to move robot
        m_drivetrain.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                            speeds.omegaRadiansPerSecond,
                            true);
    }


    /** Calculate trajectory manually with a clamped cubic spline */
    private Trajectory calculateTrajectory(double[][] points, double startAngle, double endAngle, double startSpeed,
            double endSpeed, boolean reverse, boolean rotatepath) {

        Pose2d startPose = new Pose2d(points[0][0], points[0][1], Rotation2d.fromDegrees(startAngle));
        Pose2d endPose = new Pose2d(points[points.length - 1][0], points[points.length - 1][1],
                Rotation2d.fromDegrees(endAngle));

        ArrayList<Translation2d> path = new ArrayList<>();

        // If the desired set of points contains at least one waypoint, add them to the
        // path object
        if (points.length > 2) {
            for (int i = 1; i < path.size() - 1; i++) {
                Translation2d point = new Translation2d(points[i][0], points[i][1]);
                path.add(point);
            }
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
        trajectoryConfig.setStartVelocity(startSpeed);
        trajectoryConfig.setEndVelocity(endSpeed);
        trajectoryConfig.setReversed(reverse);

        // create our trajectory
        Trajectory robotRelativeTrajectory = TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);

        // if rotating, then get transformation from current robot odometry to the path's initialpoint
        if (rotatepath)
        {
            Transform2d transform = m_odometry.getPose2d().minus(robotRelativeTrajectory.getInitialPose());
            robotRelativeTrajectory = robotRelativeTrajectory.transformBy(transform);
        }
        // we are not rotating. Only determine translation from current robot position to start position of path
        else
        {
            Translation2d translate = m_odometry.getPose2d().getTranslation().minus(robotRelativeTrajectory.getInitialPose().getTranslation());
            Rotation2d rotate = new Rotation2d(0.0);
            robotRelativeTrajectory = robotRelativeTrajectory.transformBy(new Transform2d(translate, rotate)); 
        }


        return robotRelativeTrajectory;
        // // do path rotation first (if any)
        // if (rotatepath)
        // {
        //     Translation2d temptranslate = new Translation2d(0.0,0.0);
        //     Rotation2d temprotate = m_odometry.getPose2d().getRotation();

        //     // go ahead and perform rotation
        //     robotRelativeTrajectory = robotRelativeTrajectory.transformBy(new Transform2d(temptranslate, temprotate));
        // }
        
        // // do translation next
        // Translation2d translate = new Translation2d(m_odometry.getPose2d().getX() - robotRelativeTrajectory.getInitialPose().getX(),//startPose.getX(),
        //                                        m_odometry.getPose2d().getY() - robotRelativeTrajectory.getInitialPose().getY()); //startPose.getY());
        // Rotation2d rotate = new Rotation2d(0.0);
    
        // return robotRelativeTrajectory.transformBy(new Transform2d(translate, rotate)); 
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        Translation2d zeroPoint = new Translation2d(0.0, 0.0);
        m_drivetrain.drive(zeroPoint, 0.0, true);
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        // we are finished when the time spent in this command is >= duration of path (in seconds)
        if (timer.get() >= trajectory.getTotalTimeSeconds())
            return true;
        else
            return false;
    }

} // end FollowPath command
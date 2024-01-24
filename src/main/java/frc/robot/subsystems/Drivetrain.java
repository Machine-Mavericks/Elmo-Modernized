package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.PhoenixUnsafeAccess;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


/**
 * Subsystem representing the swerve drivetrain
 */
public class Drivetrain extends SubsystemBase {
    protected final ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    public static final double updateDt = 0.02;
    

    //Useful reference: https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-builder-api.html
    
    // Helper class to ensure all constants are formatted correctly for Pheonix 6 swerve library
    // Values are set based on old constants from the SDS library
    // https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/SwerveDriveConstantsCreator.java
    public class SwerveModuleSettings {
        /** Gear ratio between drive motor and wheel. Drive reduction constants taken from the original SDS library. 
         * Reciprocal is taken to get expected format for number in pheonix library, for a better explaination, read the source code for SwerveModuleConstants*/
        public static final double DriveMotorGearRatio = 1 / MK4_L1_DriveReduction; 
        /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
        public static final double SteerMotorGearRatio = 1 / MK4_L1_SteerReduction;
        /** Wheel radius of the driving wheel in inches */
        public static final double WheelDiameter = Units.metersToInches(MK4_L1_WheelDiameter);
        /** The maximum amount of current the drive motors can apply without slippage */
        public static final double SlipCurrent = 400;


        // TODO: Figure out actual PID values to use. These were stolen from 
        // https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/Robot.java

        /** The steer motor gains */
        private static final Slot0Configs SteerMotorGains = new Slot0Configs()
        .withKP(20).withKI(0).withKD(0.05)
        .withKS(0).withKV(1).withKA(0);
        /** The drive motor gains */
        public static final Slot0Configs DriveMotorGains = new Slot0Configs()
        .withKP(0.4).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);



        /** Only option is Voltage without pro liscence */ 
        public static final ClosedLoopOutputType DriveClosedLoopOutput = ClosedLoopOutputType.Voltage; 
        public static final ClosedLoopOutputType SteerClosedLoopOutput = ClosedLoopOutputType.Voltage;


        public static final double SpeedAt12VoltsMps = MAX_VELOCITY_METERS_PER_SECOND; 

        /** True if the steering motor is reversed from the CANcoder */
        public static final boolean DriveMotorInverted = MK4_L1_DriveInverted;


        /** True if the steering motor is reversed from the CANcoder */
        public static final boolean SteerMotorInverted = MK4_L1_SteerInverted;
    }

    

            
    // value controlled on shuffleboard to stop the jerkiness of the robot by limiting its acceleration
    public GenericEntry maxAccel;
    public GenericEntry speedLimitFactor;

    public static final String CAN_BUS_NAME = "rio"; // If the drivetrain runs CANivore, change to name of desired CAN loop
    public static final int MODULE_COUNT = 4;
    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.6;
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.6;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // Swerve module physical positions
    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0);


    // All gearing values used to be supplied by the SDS library, which was discontinued
    // Values taken from https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/SdsModuleConfigurations.java
    public static final double MK4_L1_DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final double MK4_L1_SteerReduction = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double MK4_L1_WheelDiameter = 0.10033;

    public static final boolean MK4_L1_DriveInverted = true;
    public static final boolean MK4_L1_SteerInverted = false;
     

    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *      
            MK4_L1_DriveReduction *
            MK4_L1_WheelDiameter * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    /* The model representing the drivetrain's kinematics */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        FRONT_LEFT_OFFSET,
        FRONT_RIGHT_OFFSET,
        BACK_LEFT_OFFSET,
        BACK_RIGHT_OFFSET
    );
            

    // These are our modules. We set them in the constructor.
    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    /* Target chassisSpeeds (robot relative) */
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private ShuffleboardTab tab;

    // Swerve module states - contains speed(m/s) and angle for each swerve module
    SwerveModuleState[] m_states;

    // Configured in constructor
    public final double ODOMETRY_THREAD_HZ;  
    public final boolean USING_CAN_FD;

    /**
     * Create a new swerve drivetrain
     * 
     * @param frontLeftModule  Front-left swerve module
     * @param frontRightModule Front-right swerve module
     * @param backLeftModule   Back-left swerve module
     * @param backRightModule  Back-right swerve module
     * @param navx             Pigeon IMU
     */
    public Drivetrain() {
        USING_CAN_FD = CANBus.isNetworkFD(CAN_BUS_NAME);
        ODOMETRY_THREAD_HZ = USING_CAN_FD ? 250 : 100;

        tab = Shuffleboard.getTab("Drivetrain");

        resetModules(NeutralModeValue.Brake);

        /**Acceleration Limiting Slider*/
        maxAccel = tab.addPersistent("Max Acceleration", 0.05)
        .withPosition(8, 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 0.5))
        .getEntry();
        speedLimitFactor = tab.addPersistent("SpeedLimitFactor", 0.75)
        .withPosition(8, 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 0.75))
        .getEntry();
        tab.add("Reset Drivetrain", new InstantCommand(()->{resetModules(NeutralModeValue.Brake);}))
        .withPosition(0,0)
        .withSize(2, 1);

    }

    // Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
    private void resetModules(NeutralModeValue nm) {
        System.out.println("Resetting swerve modules");

        // Init Front Left Module
        SwerveModuleConstants frontLeftConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.FL_STEER_FALCON, 
                RobotMap.CANID.FL_DRIVE_FALCON, 
                RobotMap.CANID.FL_STEER_ENCODER, 
                21.79 / 360,  // -Math.toRadians(155 + 180)
                FRONT_LEFT_OFFSET.getX(),
                FRONT_LEFT_OFFSET.getY()
        );
        m_frontLeftModule = new SwerveModule(frontLeftConstants, CAN_BUS_NAME);
        m_frontLeftModule.configNeutralMode(nm);
        m_frontLeftModule.getPosition(true); // Appears to refresh internal position used for optimization

        // Init Front Right Module
        SwerveModuleConstants frontRightConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.FR_STEER_FALCON, 
                RobotMap.CANID.FR_DRIVE_FALCON, 
                RobotMap.CANID.FR_STEER_ENCODER, 
                -274.92 / 360, // Degrees converted to rotations
                FRONT_RIGHT_OFFSET.getX(),
                FRONT_RIGHT_OFFSET.getY()
        );
        m_frontRightModule = new SwerveModule(frontRightConstants, CAN_BUS_NAME);
        m_frontRightModule.configNeutralMode(nm);
        m_frontRightModule.getPosition(true); // Appears to refresh internal position used for optimization


        // Init Back Left Module
        SwerveModuleConstants backLeftConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.BL_STEER_FALCON, 
                RobotMap.CANID.BL_DRIVE_FALCON, 
                RobotMap.CANID.BL_STEER_ENCODER, 
                -17.22 / 360, 
                BACK_LEFT_OFFSET.getX(),
                BACK_LEFT_OFFSET.getY()
        );
        m_backLeftModule = new SwerveModule(backLeftConstants, CAN_BUS_NAME);
        m_backLeftModule.configNeutralMode(nm);
        m_backLeftModule.getPosition(true); // Appears to refresh internal position used for optimization


        // Init Back Right Module
        SwerveModuleConstants backRightConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.BR_STEER_FALCON, 
                RobotMap.CANID.BR_DRIVE_FALCON, 
                RobotMap.CANID.BR_STEER_ENCODER, 
                47.9 / 360, 
                BACK_RIGHT_OFFSET.getX(),
                BACK_RIGHT_OFFSET.getY()
        );
        m_backRightModule = new SwerveModule(backRightConstants, CAN_BUS_NAME);
        m_backRightModule.configNeutralMode(nm);
        m_backRightModule.getPosition(true); // Appears to refresh internal position used for optimization
    }

    // It seems there is already a factory for SwerveModuleConstants
    public static SwerveModuleConstants CreateSwerveModuleConstants(
        int steerId,
        int driveId,
        int cancoderId,
        double cancoderOffset,
        double locationX,
        double locationY

    ){
        SwerveModuleConstants constants = new SwerveModuleConstants()
        .withSteerMotorId(steerId)
        .withDriveMotorId(driveId)
        .withCANcoderId(cancoderId)
        .withCANcoderOffset(cancoderOffset)
        .withLocationX(locationX)
        .withLocationY(locationY)
        .withDriveMotorGearRatio(SwerveModuleSettings.DriveMotorGearRatio)
        .withSteerMotorGearRatio(SwerveModuleSettings.SteerMotorGearRatio)
        .withWheelRadius(SwerveModuleSettings.WheelDiameter / 2)
        .withSlipCurrent(SwerveModuleSettings.SlipCurrent)
        .withSteerMotorGains(SwerveModuleSettings.SteerMotorGains)
        .withDriveMotorGains(SwerveModuleSettings.DriveMotorGains)
        .withSteerMotorClosedLoopOutput(SwerveModuleSettings.SteerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(SwerveModuleSettings.DriveClosedLoopOutput)
        .withSpeedAt12VoltsMps(SwerveModuleSettings.SpeedAt12VoltsMps)
        .withSteerMotorInverted(SwerveModuleSettings.SteerMotorInverted)
        .withDriveMotorInverted(SwerveModuleSettings.DriveMotorInverted);

        return constants;
    }
    /**
     * Control the drivetrain
     * 
     * @param translation   X/Y translation, in meters per second
     * @param rotation      Rotation, in radians per second
     * @param fieldOriented Boolean indicating if directions are field- or
     *                      robot-oriented
     */
    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {

        // correct axes of drive - determined from field testing
        // Feb 10 2022
        // flip sign of y axis speed
        // flip sign of rotation speed
        Translation2d newtranslation = new Translation2d(translation.getX(),
                -translation.getY());
        Double newrotation = -rotation;

        // determine chassis speeds
        if (fieldOriented) {
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(newtranslation.getX(),
                    newtranslation.getY(),
                    newrotation,
                    Rotation2d.fromDegrees(RobotContainer.gyro.getYaw()));
        } else {
            m_chassisSpeeds = new ChassisSpeeds(newtranslation.getX(),
                    newtranslation.getY(),
                    newrotation);
        }
    }

    @Override
    public void periodic() {
        //TODO: Replace with odometry thread
        // m_frontLeftModule.getPosition(true);
        // m_frontRightModule.getPosition(true);
        // m_backLeftModule.getPosition(true);
        m_backRightModule.getPosition(true);

        // Look ahead in time one control loop
        // Pose2d robotDeltaPose = new Pose2d(m_chassisSpeeds.vxMetersPerSecond * updateDt, m_chassisSpeeds.vyMetersPerSecond * updateDt, Rotation2d.fromRadians(m_chassisSpeeds.omegaRadiansPerSecond * updateDt));
        // // Black magic
        // Twist2d velocity_twist2d = new Pose2d().log(robotDeltaPose); // Twist between two poses .log is relative to zeroed pose. I still don't know calculus, so understanding this is a bit of a problem...
        // ChassisSpeeds discretizedChassisSpeeds = new ChassisSpeeds(velocity_twist2d.dx / updateDt, velocity_twist2d.dy / updateDt, velocity_twist2d.dtheta / updateDt);
        
        /* Above can be replaced with new WPILib call to discretize speeds */
        ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(m_chassisSpeeds, updateDt);

        m_states = m_kinematics.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);
        SmartDashboard.putString("Processed Speeds", discretizedChassisSpeeds.toString());

        SmartDashboard.putString("FrontLeftState", m_frontLeftModule.getCurrentState().toString());
        SmartDashboard.putString("FrontRightState", m_frontRightModule.getCurrentState().toString());
        SmartDashboard.putString("BackLeftState", m_backLeftModule.getCurrentState().toString());
        SmartDashboard.putString("BackRightState", m_backRightModule.getCurrentState().toString());

        // TODO: OpenLoopVoltage seems to match SDS library best, but is open loop
        // For auto consistency we should aim for closed loop control
        DriveRequestType driveRequestType = DriveRequestType.OpenLoopVoltage;


        // Steer request type defaults correctly to MotionMagic
        m_frontLeftModule.apply(m_states[0], driveRequestType); 
        m_frontRightModule.apply(m_states[1], driveRequestType);
        m_backLeftModule.apply(m_states[2], driveRequestType);
        m_backRightModule.apply(m_states[3], driveRequestType);        
    }

    // -------------------- Kinematics and Swerve Module Status Public Access Methods --------------------

    /** Returns kinematics of drive system */
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Returns speed and angle status of all swerve modules
     * Returns array of length of of SwerveModuleStates
     */
    public SwerveModuleState[] getSwerveStates() {

        // create array of module states to return
        SwerveModuleState[] states = new SwerveModuleState[4];
        
        states[0] = m_frontLeftModule.getCurrentState();

        states[1] = m_frontRightModule.getCurrentState();

        states[2] = m_backLeftModule.getCurrentState();

        states[3] = m_backRightModule.getCurrentState();

        return states;
    }

    /** Returns swerve module positions, optionally, latency compensation / prediction can be enabled. YMMV */
    public SwerveModulePosition[] getSwervePositions(boolean LatencyCompensate){
        // create array of module positions to return
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        if (LatencyCompensate) {
            positions[0] = m_frontLeftModule.getPosition(false);
            positions[1] = m_frontRightModule.getPosition(false);
            positions[2] = m_backLeftModule.getPosition(false);
            positions[3] = m_backRightModule.getPosition(false);
        }else{
            positions[0] = m_frontLeftModule.getCachedPosition();
            positions[1] = m_frontRightModule.getCachedPosition();
            positions[2] = m_backLeftModule.getCachedPosition();
            positions[3] = m_backRightModule.getCachedPosition();
        }
       

        return positions;
    }

    /* Runs odometry on another thread to reduce latency. Modified from SwerveDrivetrain.java in Phoenix 6 */
    public class OdometryUpdateThread {
        /**
         * Priority level to set the DAQ thread to.
         * This is a value between 0 and 99, with 99 indicating higher priority and 0 indicating lower priority.
         */
        protected static final int START_THREAD_PRIORITY = 1; // Testing shows 1 (minimum realtime) is sufficient for tighter
                                                            // odometry loops.
                                                            // If the odometry period is far away from the desired frequency,
                                                            // increasing this may help

        protected final Thread m_thread;
        protected volatile boolean m_running = false;

        protected final BaseStatusSignal[] m_allSignals;

        protected final MedianFilter peakRemover = new MedianFilter(3);
        protected final LinearFilter lowPass = LinearFilter.movingAverage(50);
        protected double lastTime = 0;
        protected double currentTime = 0;
        protected double averageLoopTime = 0;

        public OdometryUpdateThread() {
            m_thread = new Thread(this::run);
            /* Mark this thread as a "daemon" (background) thread
             * so it doesn't hold up program shutdown */
            m_thread.setDaemon(true);

            /* 4 signals for each module + 2 for Pigeon2 */
            m_allSignals = new BaseStatusSignal[(MODULE_COUNT * 4) + 2];

            AddModuleSignals(m_frontLeftModule, 0);
            AddModuleSignals(m_frontRightModule, 1);
            AddModuleSignals(m_backLeftModule, 2);
            AddModuleSignals(m_backRightModule, 3);
            m_allSignals[i].setUpdateFrequency(MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)

            
        }

        private void AddModuleSignals(SwerveModule module, int index){
            var signals = PhoenixUnsafeAccess.getSwerveSignals(module); // Dirty hack part 1
            m_allSignals[(index * 4) + 0] = signals[0];
            m_allSignals[(index * 4) + 1] = signals[1];
            m_allSignals[(index * 4) + 2] = signals[2];
            m_allSignals[(index * 4) + 3] = signals[3];
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            m_running = true;
            m_thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         *
         * @param millis The time to wait in milliseconds
         */
        public void stop(long millis) {
            m_running = false;
            try {
                m_thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        public void run() {
            /* Make sure all signals update at the correct update frequency */
            BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_THREAD_HZ, m_allSignals);
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

            /* Run as fast as possible, our signals will control the timing */
            while (m_running) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                if (USING_CAN_FD) {
                    status = BaseStatusSignal.waitForAll(2.0 / ODOMETRY_THREAD_HZ, m_allSignals);
                } else {
                    /* Wait for the signals to update */
                    Timer.delay(1.0 / ODOMETRY_THREAD_HZ);
                    status = BaseStatusSignal.refreshAll(m_allSignals);
                }

                try {
                    m_stateLock.writeLock().lock();

                    lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();
                    /* We don't care about the peaks, as they correspond to GC events, and we want the period generally low passed */
                    averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));


                    /* Now update odometry */
                    /* Keep track of the change in encoder rotations */
                    /* Status signals have automatically already been updated, and don't need refreshing */
                    SwerveModulePosition[] positions = new SwerveModulePosition[4];

                    positions[0] = m_frontLeftModule.getPosition(false);
                    positions[1] = m_frontRightModule.getPosition(false);
                    positions[2] = m_backLeftModule.getPosition(false);
                    positions[3] = m_backRightModule.getPosition(false);


                    // Gyro latency compensation. Using a navX, so isn't all that easy, disabled for now
                    // double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                    //         m_yawGetter, m_angularVelocity);

                    /* Keep track of previous and current pose to account for the carpet vector */
                    m_odometry.update(Rotation2d.fromDegrees(RobotContainer.gyro.getYaw()), positions);

                    /* And now that we've got the new odometry, update the controls */
                    m_requestParameters.currentPose = m_odometry.getEstimatedPosition();
                    m_requestParameters.kinematics = m_kinematics;
                    m_requestParameters.swervePositions = positions;
                    m_requestParameters.timestamp = currentTime;
                    m_requestParameters.updatePeriod = 1.0 / UpdateFrequency;

                    //m_requestToApply.apply(m_requestParameters, Modules);

                    /* Update our cached state with the newly updated data */
                    m_cachedState.Pose = m_odometry.getEstimatedPosition();
                    m_cachedState.OdometryPeriod = averageLoopTime;

                    if (m_cachedState.ModuleStates == null) {
                        m_cachedState.ModuleStates = new SwerveModuleState[Modules.length];
                    }
                    if (m_cachedState.ModuleTargets == null) {
                        m_cachedState.ModuleTargets = new SwerveModuleState[Modules.length];
                    }
                    for (int i = 0; i < Modules.length; ++i) {
                        m_cachedState.ModuleStates[i] = Modules[i].getCurrentState();
                        m_cachedState.ModuleTargets[i] = Modules[i].getTargetState();
                    }

                } finally {
                    m_stateLock.writeLock().unlock();
                }
            }
        }

        public OdometryData getOdometryData(){
            return m_odometryData;
        }
    } // end class OdometryUpdateThread

} // end class Drivetrain
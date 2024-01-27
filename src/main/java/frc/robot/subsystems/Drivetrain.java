package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.PhoenixUnsafeAccess;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.Utils;


/**
 * Subsystem representing the swerve drivetrain
 */
public class Drivetrain extends SubsystemBase {
    public static final double updateDt = 0.02;
    
    //Useful reference: https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-builder-api.html
    
    // Helper class to ensure all constants are formatted correctly for Pheonix 6 swerve library
    // Values are set based on old constants from the SDS library
    // https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/SwerveDriveConstantsCreator.java
    public class FormattedSwerveModuleSettings {
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
        .withKP(0.1).withKI(0).withKD(0) // TODO: Tune this value
        .withKV(0.11);


        /** Only option is Voltage without pro liscence */ 
        public static final ClosedLoopOutputType DriveClosedLoopOutput = ClosedLoopOutputType.Voltage; 
        public static final ClosedLoopOutputType SteerClosedLoopOutput = ClosedLoopOutputType.Voltage;


        public static final double SpeedAt12VoltsMps = MAX_VELOCITY_METERS_PER_SECOND; 

        /** True if the driving motor is reversed */
        public static final boolean DriveMotorInverted = MK4_L1_DriveInverted;


        /** True if the steering motor is reversed from the CANcoder */
        public static final boolean SteerMotorInverted = MK4_L1_SteerInverted;
    }


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



    // Whether motors should try to brake
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

    // -------------------- Swerve Module Data Arrays --------------------
    
    /*
     * Swerve module data is ordered as follows
     * FrontLeft
     * FrontRight
     * BackLeft
     * BackRight
     */
    // Current swerve module states - contains speed(m/s) and angle for each swerve module
    SwerveModuleState[] m_states;
    // Current swerve module states - contains speed(m/s) and angle for each swerve module
    SwerveModuleState[] m_targetStates;
    // Current swerve module positions - contains number of meters wheel has rotated as well as module angle
    SwerveModulePosition[] m_positions;
    // Odometry signals
    BaseStatusSignal[] m_allSignals;
    // The swerve modules themselves
    private SwerveModule[] m_swerveModules;


    // The model representing the drivetrain's kinematics
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        FRONT_LEFT_OFFSET,
        FRONT_RIGHT_OFFSET,
        BACK_LEFT_OFFSET,
        BACK_RIGHT_OFFSET
    );
    
    // Target chassisSpeeds (robot relative)
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Odometry settings configured in constructor
    public final double ODOMETRY_HZ;  
    public final boolean USING_CAN_FD;

    // Shuffleboard classes
    private ShuffleboardTab tab;
    // value controlled on shuffleboard to stop the jerkiness of the robot by limiting its acceleration
    public GenericEntry maxAccel;
    public GenericEntry speedLimitFactor;

    
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
        ODOMETRY_HZ = USING_CAN_FD ? 250 : 100;

        tab = Shuffleboard.getTab("Drivetrain");

        resetModules(NEUTRAL_MODE);

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
        tab.add("Reset Drivetrain", new InstantCommand(()->{resetModules(NEUTRAL_MODE);}))
        .withPosition(0,0)
        .withSize(2, 1);
    }

    // Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
    private void resetModules(NeutralModeValue nm) {
        System.out.println("Resetting swerve modules");

        // Init all arrays
        m_swerveModules = new SwerveModule[MODULE_COUNT];
        m_positions = new SwerveModulePosition[MODULE_COUNT];
        m_states = new SwerveModuleState[MODULE_COUNT];
        m_targetStates = new SwerveModuleState[MODULE_COUNT];

        m_allSignals = new BaseStatusSignal[MODULE_COUNT * 4];
        
        // Init Front Left Module
        SwerveModuleConstants frontLeftConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.FL_STEER_FALCON, 
                RobotMap.CANID.FL_DRIVE_FALCON, 
                RobotMap.CANID.FL_STEER_ENCODER, 
                21.79 / 360,  // -Math.toRadians(155 + 180)
                FRONT_LEFT_OFFSET.getX(),
                FRONT_LEFT_OFFSET.getY()
        );
        m_swerveModules[0] = new SwerveModule(frontLeftConstants, CAN_BUS_NAME);

        // Init Front Right Module
        SwerveModuleConstants frontRightConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.FR_STEER_FALCON, 
                RobotMap.CANID.FR_DRIVE_FALCON, 
                RobotMap.CANID.FR_STEER_ENCODER, 
                -274.92 / 360, // Degrees converted to rotations
                FRONT_RIGHT_OFFSET.getX(),
                FRONT_RIGHT_OFFSET.getY()
        );
        m_swerveModules[1] = new SwerveModule(frontRightConstants, CAN_BUS_NAME);


        // Init Back Left Module
        SwerveModuleConstants backLeftConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.BL_STEER_FALCON, 
                RobotMap.CANID.BL_DRIVE_FALCON, 
                RobotMap.CANID.BL_STEER_ENCODER, 
                -17.22 / 360, 
                BACK_LEFT_OFFSET.getX(),
                BACK_LEFT_OFFSET.getY()
        );
        m_swerveModules[2] = new SwerveModule(backLeftConstants, CAN_BUS_NAME);


        // Init Back Right Module
        SwerveModuleConstants backRightConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.BR_STEER_FALCON, 
                RobotMap.CANID.BR_DRIVE_FALCON, 
                RobotMap.CANID.BR_STEER_ENCODER, 
                47.9 / 360, 
                BACK_RIGHT_OFFSET.getX(),
                BACK_RIGHT_OFFSET.getY()
        );
        m_swerveModules[3] = new SwerveModule(backRightConstants, CAN_BUS_NAME);
        

        // Perform initial configuration on all swerve modules
        for (int i = 0; i < m_swerveModules.length; i++){
            SwerveModule module = m_swerveModules[i];

            module.configNeutralMode(nm);
            AddModuleSignals(module, i);
            m_positions[i] = m_swerveModules[i].getPosition(true); // Appears to refresh internal position used for optimization
            m_states[i] = new SwerveModuleState(); // Initialize to blank states at the beginning, will be overwritten in first periodic loop
            m_targetStates[i] = new SwerveModuleState();
        }


        /* Make sure all signals update at the correct update frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_HZ, m_allSignals);
    }

    private void AddModuleSignals(SwerveModule module, int index){
        var signals = PhoenixUnsafeAccess.getSwerveSignals(module); // Dirty hack
        m_allSignals[(index * 4) + 0] = signals[0];
        m_allSignals[(index * 4) + 1] = signals[1];
        m_allSignals[(index * 4) + 2] = signals[2];
        m_allSignals[(index * 4) + 3] = signals[3];
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
        .withDriveMotorGearRatio(FormattedSwerveModuleSettings.DriveMotorGearRatio)
        .withSteerMotorGearRatio(FormattedSwerveModuleSettings.SteerMotorGearRatio)
        .withWheelRadius(FormattedSwerveModuleSettings.WheelDiameter / 2)
        .withSlipCurrent(FormattedSwerveModuleSettings.SlipCurrent)
        .withSteerMotorGains(FormattedSwerveModuleSettings.SteerMotorGains)
        .withDriveMotorGains(FormattedSwerveModuleSettings.DriveMotorGains)
        .withSteerMotorClosedLoopOutput(FormattedSwerveModuleSettings.SteerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(FormattedSwerveModuleSettings.DriveClosedLoopOutput)
        .withSpeedAt12VoltsMps(FormattedSwerveModuleSettings.SpeedAt12VoltsMps)
        .withSteerMotorInverted(FormattedSwerveModuleSettings.SteerMotorInverted)
        .withDriveMotorInverted(FormattedSwerveModuleSettings.DriveMotorInverted);

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
                translation.getY());
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
        // Ensure sensor readings & pose estimator are up to date
        updateOdometryData();

        // Look ahead in time one control loop and adjust
        
        // 4738's implementation. With fudge factor to account for latency
        ChassisSpeeds discretizedChassisSpeeds = DiscretizeChassisSpeeds(m_chassisSpeeds, updateDt);

        // WPILib's implementation
        //ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(m_chassisSpeeds, updateDt);

        SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, MAX_VELOCITY_METERS_PER_SECOND);
        SmartDashboard.putString("Unprocessed Speeds", m_chassisSpeeds.toString());
        SmartDashboard.putString("Processed Speeds", discretizedChassisSpeeds.toString());

        // Prepared debugging for closed loop drive motor
        SmartDashboard.putString("FrontLeftState", m_swerveModules[0].getCurrentState().toString());
        SmartDashboard.putString("FrontLeftDriveError", String.valueOf(m_swerveModules[0].getDriveMotor().getClosedLoopError().getValue()));
        SmartDashboard.putString("FrontLeftDriveTarget", String.valueOf(m_swerveModules[0].getDriveMotor().getClosedLoopReference().getValue()));
        SmartDashboard.putString("FrontLeftDriveCurrent", String.valueOf(m_swerveModules[0].getDriveMotor().getVelocity().getValue()));
        SmartDashboard.putString("FrontLeftTargetState", m_swerveModules[0].getTargetState().toString());

        // TODO: OpenLoopVoltage seems to match SDS library best, but is open loop
        // For auto consistency we should aim for closed loop control
        DriveRequestType driveRequestType = DriveRequestType.OpenLoopVoltage;


        // Steer request type defaults correctly to MotionMagic
        for (int i = 0; i < m_swerveModules.length; i++){
            m_swerveModules[i].apply(targetStates[i], driveRequestType); 
        }     
    }

    // Fudge compensation for angle included, other teams have had success with a factor of 4
    // public static ChassisSpeeds DiscretizeChassisSpeeds(ChassisSpeeds speeds, double dt, double rotationCompFactor){ 
    //     var desiredDeltaPose = new Pose2d(
    //         speeds.vxMetersPerSecond * dt, 
    //         speeds.vyMetersPerSecond * dt, 
    //         new Rotation2d(speeds.omegaRadiansPerSecond * dt * rotationCompFactor)
    //     );
    //     var twist = new Pose2d().log(desiredDeltaPose);

    //     return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    // }

    public static ChassisSpeeds DiscretizeChassisSpeeds(ChassisSpeeds speeds, double dt){ 
        var futureRobotPose = new Pose2d(
            speeds.vxMetersPerSecond * dt, 
            speeds.vyMetersPerSecond * dt, 
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * -20)
        );
        var twist = Utils.log(futureRobotPose);
        SmartDashboard.putString("Twist Speeds", twist.toString());
        // FLIPPED COORDINATE SYSTEM
        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    public void updateOdometryData(){
        // Refresh odometry data
        BaseStatusSignal.refreshAll(m_allSignals);

        // Update / fill swerve related data
        for (int i = 0; i < m_swerveModules.length; i++){
            m_positions[i] = m_swerveModules[i].getPosition(false);
            m_states[i] = m_swerveModules[i].getCurrentState();
            m_targetStates[i] = m_swerveModules[i].getTargetState();
        }

        // Update pose estimator with odometry data
        RobotContainer.odometry.updateOdometry();
    }

    // -------------------- Kinematics and Swerve Module Status Public Access Methods --------------------

    /** Returns kinematics of drive system */
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Returns speed and angle status of all swerve modules
     * @return array of length of number of swerve modules
     */
    public SwerveModuleState[] getSwerveStates() {
        return m_states;
    }

    /**
     * Returns targeted speed and angle status of all swerve modules
     * @return array of length of number of swerve modules
     */
    public SwerveModuleState[] getTargetSwerveStates() {
        return m_targetStates;
    }

    /** 
     * Returns swerve module positions
     * @return array of length of number of swerve modules
     */
    public SwerveModulePosition[] getSwervePositions(){
        return m_positions;
    }

    
} // end class Drivetrain
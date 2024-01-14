package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.ctre.phoenix6.configs.Slot0Configs;


/**
 * Subsystem representing the swerve drivetrain
 */
public class Drivetrain extends SubsystemBase {

    // Useful:
    // https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/SwerveDriveConstantsCreator.java
    public class SwerveModuleSettings {
        /** Gear ratio between drive motor and wheel */
        public final doublePROBLEM DriveMotorGearRatio; // TODO: This is not the ratio the swerve constructor wants
        /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
        public final double SteerMotorGearRatio;
        /** Wheel radius of the driving wheel in inches */
        public final double wheelDiameter;
        /** The maximum amount of current the drive motors can apply without slippage */
        public final double SlipCurrent = 400;

        /** The steer motor gains */
        public final Slot0Configs SteerMotorGains = 2;
        /** The drive motor gains */
        public final Slot0Configs DriveMotorGains = 2;

        /** True if the steering motor is reversed from the CANcoder */
        public final boolean SteerMotorInverted;

        public SwerveModuleSettings(double driveMotorGearRatio, double steerMotorGearRatio, double wheelDiameter, boolean steerMotorInverted){
            this.DriveMotorGearRatio = driveMotorGearRatio;
            this.SteerMotorGearRatio = steerMotorGearRatio;
            this.wheelDiameter = wheelDiameter;

            this.SteerMotorInverted = steerMotorInverted;
        }
    }

    

            
    // value controlled on shuffleboard to stop the jerkiness of the robot by limiting its acceleration
    public GenericEntry maxAccel;
    public GenericEntry speedLimitFactor;

    public static final String CAN_BUS_NAME = "Elmo"; // TODO: ?????

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.6;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.6;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;



    // Front left
    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
    // Front right
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0);
    // Back left
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
    // Back right
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0);


    // All gearing values used to be supplied by the SDS library, which was discontinued
    // Values taken from https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/SdsModuleConfigurations.java
    public static final double MK4_L1_DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final double MK4_L1_SteerReduction = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double MK4_L1_WheelDiameter = 0.10033;

    public static final boolean MK4_L1_DriveInverted = true;
    public static final boolean MK4_L1_SteerInverted = true;

    public static SwerveModuleSettings SWERVE_SETTINGS;
     

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
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *      
            MK4_L1_DriveReduction *
            MK4_L1_WheelDiameter * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    /**
     * The model representing the drivetrain's kinematics
     */
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

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private ShuffleboardTab tab;

    // Swerve module states - contains speed(m/s) and angle for each swerve module
    SwerveModuleState[] m_states;

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
        SWERVE_SETTINGS = new SwerveModuleSettings(MK4_L1_DriveReduction, MK4_L1_SteerReduction, MK4_L1_WheelDiameter, false);
        // SmartDashboard.putData("Field", m_field);

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
        //final Mk4SwerveModuleHelper.GearRatio DRIVE_RATIO = Mk4SwerveModuleHelper.GearRatio.L1;

        // Init Front Left Module
        SwerveModuleConstants frontLeftConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.FL_STEER_FALCON, 
                RobotMap.CANID.FL_DRIVE_FALCON, 
                RobotMap.CANID.FL_STEER_ENCODER, 
                -Math.toRadians(155 + 180), 
                FRONT_LEFT_OFFSET.getX(),
                FRONT_LEFT_OFFSET.getY()
        );
        m_frontLeftModule = new SwerveModule(frontLeftConstants, CAN_BUS_NAME);
        m_frontLeftModule.configNeutralMode(nm);


        // Init Front Right Module
        SwerveModuleConstants frontRightConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.FR_STEER_FALCON, 
                RobotMap.CANID.FR_DRIVE_FALCON, 
                RobotMap.CANID.FR_STEER_ENCODER, 
                -Math.toRadians(94 + 180), 
                FRONT_RIGHT_OFFSET.getX(),
                FRONT_RIGHT_OFFSET.getY()
        );
        m_frontRightModule = new SwerveModule(frontRightConstants, CAN_BUS_NAME);
        m_frontRightModule.configNeutralMode(nm);


        // Init Back Left Module
        SwerveModuleConstants backLeftConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.BL_STEER_FALCON, 
                RobotMap.CANID.BL_DRIVE_FALCON, 
                RobotMap.CANID.BL_STEER_ENCODER, 
                -Math.toRadians(200 + 180), 
                BACK_LEFT_OFFSET.getX(),
                BACK_LEFT_OFFSET.getY()
        );
        m_backLeftModule = new SwerveModule(backLeftConstants, CAN_BUS_NAME);
        m_backLeftModule.configNeutralMode(nm);


        // Init Back Right Module
        SwerveModuleConstants backRightConstants = CreateSwerveModuleConstants(
                RobotMap.CANID.BR_STEER_FALCON, 
                RobotMap.CANID.BR_DRIVE_FALCON, 
                RobotMap.CANID.BR_STEER_ENCODER, 
                -Math.toRadians(135 + 180), 
                BACK_RIGHT_OFFSET.getX(),
                BACK_RIGHT_OFFSET.getY()
        );
        m_backRightModule = new SwerveModule(backRightConstants, CAN_BUS_NAME);
        m_backRightModule.configNeutralMode(nm);

        // m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        //         tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(0, 0),
        //         DRIVE_RATIO, RobotMap.CANID.FL_DRIVE_FALCON, RobotMap.CANID.FL_STEER_FALCON,
        //         RobotMap.CANID.FL_STEER_ENCODER, -Math.toRadians(155 + 180));
        

        // m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
        //         tab.getLayout("Front right Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(2, 0),
        //         DRIVE_RATIO, RobotMap.CANID.FR_DRIVE_FALCON, RobotMap.CANID.FR_STEER_FALCON,
        //         RobotMap.CANID.FR_STEER_ENCODER, -Math.toRadians(94 + 180));

        // m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        //         tab.getLayout("Back Left Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(4, 0),
        //         DRIVE_RATIO, RobotMap.CANID.BL_DRIVE_FALCON, RobotMap.CANID.BL_STEER_FALCON,
        //         RobotMap.CANID.BL_STEER_ENCODER, -Math.toRadians(200 + 180));

        // m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        //         tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(6, 0),
        //         DRIVE_RATIO, RobotMap.CANID.BR_DRIVE_FALCON, RobotMap.CANID.BR_STEER_FALCON,
        //         RobotMap.CANID.BR_STEER_ENCODER, -Math.toRadians(135 + 180));
        

        // temp1.setNeutralMode(nm);
        // temp2.setNeutralMode(nm);
        // temp3.setNeutralMode(nm);
        // temp4.setNeutralMode(nm);
    }

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
        .withDriveMotorGearRatio(SWERVE_SETTINGS.DriveMotorGearRatio)
        .withSteerMotorGearRatio(SWERVE_SETTINGS.SteerMotorGearRatio)
        .withWheelRadius(SWERVE_SETTINGS.wheelDiameter / 2)
        .withSlipCurrent(SWERVE_SETTINGS.SlipCurrent)
        .withSteerMotorGains(SWERVE_SETTINGS.SteerMotorGains)
        .withDriveMotorGains(SWERVE_SETTINGS.DriveMotorGains)
        .withSteerMotorInverted(SWERVE_SETTINGS.SteerMotorInverted);

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

        // not sure what this line was intended to do. KN Feb 11/2022
        // rotation *= 2.0 / Math.hypot(WHEELBASE_METERS, TRACKWIDTH_METERS);

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

        m_states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);
        SmartDashboard.putString("Speeds", m_chassisSpeeds.toString());

        
        // m_frontLeftModule.set(m_states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        //         m_states[0].angle.getRadians());
        // m_frontRightModule.set(m_states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        //         m_states[1].angle.getRadians());
        // m_backLeftModule.set(m_states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        //         m_states[2].angle.getRadians());
        // m_backRightModule.set(m_states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        //         m_states[3].angle.getRadians());


        // Theoretically fine
        m_frontLeftModule.apply(m_states[0], DriveRequestType.Velocity);
        m_frontRightModule.apply(m_states[1], DriveRequestType.Velocity);
        m_backLeftModule.apply(m_states[2], DriveRequestType.Velocity);
        m_backRightModule.apply(m_states[3], DriveRequestType.Velocity);


        
    }

    // -------------------- Kinematics and Swerve Module Status Public Access
    // Methods --------------------

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

    /* Exists because a bunch of updated functions take SwerveModulePositions now?? */
    public SwerveModulePosition[] getSwervePositions(){
        // create array of module positions to return
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = m_frontLeftModule.getCachedPosition();

        positions[1] = m_frontRightModule.getCachedPosition();

        positions[2] = m_backLeftModule.getCachedPosition();

        positions[3] = m_backRightModule.getCachedPosition();

        return positions;
    }

} // end class Drivetrain
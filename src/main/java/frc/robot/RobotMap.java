package frc.robot;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the
 * subsystems.
 */
public class RobotMap {

    /**
     * Inner class containing CANIDs
     */
    public static class CANID {
        /** CAN ID for front-left drive falcon */
        public static final int FL_DRIVE_FALCON = 3;
        /** CAN ID for front-left steer falcon */
        public static final int FL_STEER_FALCON = 4;
        /** CAN ID for front-left steer encoder */
        public static final int FL_STEER_ENCODER = 12;
        /** CAN ID for front-right drive falcon */
        public static final int FR_DRIVE_FALCON = 5;
        /** CAN ID for front-right steer falcon */
        public static final int FR_STEER_FALCON = 6;
        /** CAN ID for front-left steer encoder */
        public static final int FR_STEER_ENCODER = 11;
        /** CAN ID for back-left drive falcon */
        public static final int BL_DRIVE_FALCON = 1;
        /** CAN ID for back-left steer falcon */
        public static final int BL_STEER_FALCON = 2;
        /** CAN ID for front-left steer encoder */
        public static final int BL_STEER_ENCODER = 9;
        /** CAN ID for back-right drive falcon */
        public static final int BR_DRIVE_FALCON = 7;
        /** CAN ID for back-right steer falcon */
        public static final int BR_STEER_FALCON = 8;
        /** CAN ID for front-left steer encoder */
        public static final int BR_STEER_ENCODER  = 10;
        /** CAN ID for left lifter motor */
        public static final int LIFTER_FALCON = 14;

        /** CAN ID for right shooter motor */
        public static final int RIGHT_SHOOTER_FALCON = 16;
        /** CAN ID for left shooter motor */
        public static final int LEFT_SHOOTER_FALCON = 17;
        /** CAN ID for top shooter motor */
        public static final int TOP_SHOOTER_FALCON = 20;
        /** CAN ID for left climber motor */
        public static final int CLIMBER_FALCON = 18;
        
        /** CAN ID for intake motor */
        public static final int INTAKE_FALCON = 13;
    }

    public static class VISION_TARGETING {
    /** minimum area for finding ball */
    public static final double MIN_BALL_DETECTION_AREA = 0.0;
    /** minimum horizontal length of ball */
    public static final double MIN_BALL_VERT_SIZE = 0.0;
    }
    
    public static final int LIFTER_LIMIT_ID = 1;
    public static final int INTAKE_LIMIT_ID = 2;

    public static final double BALL_LIFTER_SPEED = -3161;

    public static class PneumaticsChannel {
    }

    public static class PWMPorts {
        /** PWM Port for led strip */
        public static final int LED_STRIP1 = 0;
        public static final int LED_BLINKIN = 3;
        public static final int CAMERA_SERVO_ID = 2;
        public static final int SHOOTER_SERVO_ID = 4;
    }  

    /**
     * Inner class containing odometry constants
     */
    public static class ODOMETRY {
        /** feed-forward gain */
        public static final int ksVolts = 3;
        /** feed-forward gain */
        public static final int kvVoltSecondsPerMeter = 3;
        /** feed-forward gain */
        public static final int kaVoltSecondsSquaredPerMeter = 3;
        /** robot max speed */
        public static final int kMaxSpeedMetersPerSecond = 3;
        /** robot max acceleration */
        public static final int kMaxAccelerationMetersPerSecondSquared = 3;
        /** proportional gain */
        public static final int kPDriveVel = 3;
    }
    

    /**
     * Function to initialise hardware
     * Should be called in {@link Robot#robotInit()}
     */
    public static void Init() {
    }
}
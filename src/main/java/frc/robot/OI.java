package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the subsystems.
 */
public class OI {

    static double newXInput = 0.0;
    static double newYInput = 0.0;
    static double prevXInput = 0.0;
    static double prevYInput = 0.0;

// TODO: ADD SLOW DRIVE BUTTON

    public static double getXDriveInput(){
        
        double speedLimitFactor = RobotContainer.drivetrain.speedLimitFactor.getDouble(1.0);
        // read new input from controller
        prevXInput = newXInput;
        // read new input from controller
        newXInput = OI.driverController.getLeftX();
        // implement deadzoning
        newXInput = Math.abs(newXInput) > 0.1 ? newXInput : 0;
        // read max accel from shuffleboard
        double maxAccel = RobotContainer.drivetrain.maxAccel.getDouble(0.02);
        // limit the acceleration
        newXInput = (newXInput - prevXInput) > maxAccel ? prevXInput + maxAccel : newXInput;
        newXInput = (newXInput - prevXInput) < -1 * maxAccel ? prevXInput - maxAccel : newXInput;
        return ((driverController.getRightTriggerAxis() >= 0.75) ? newXInput * 0.20 : newXInput)*speedLimitFactor;
    }

    public static double getYDriveInput(){
        
        double speedLimitFactor = RobotContainer.drivetrain.speedLimitFactor.getDouble(1.0);
        // read new input from controller
        prevYInput = newYInput;
        // read new input from controller
        newYInput = OI.driverController.getLeftY();
        // implement deadzoning
        newYInput = Math.abs(newYInput) > 0.1 ? newYInput : 0;
        // read max accel from shuffleboard
        double maxAccel = RobotContainer.drivetrain.maxAccel.getDouble(0.02);
        // limit the acceleration
        newYInput = (newYInput - prevYInput) > maxAccel ? prevYInput + maxAccel : newYInput;
        newYInput = (newYInput - prevYInput) < -1 * maxAccel ? prevYInput - maxAccel : newYInput;
        return ((driverController.getRightTriggerAxis() >= 0.75) ? newYInput * 0.20 : newYInput)*speedLimitFactor;
    }

    public static double getRotDriveInput(){
        
        double speedLimitFactor = RobotContainer.drivetrain.speedLimitFactor.getDouble(1.0);
        double rotInput = driverController.getRightX()*speedLimitFactor;
        rotInput = Math.abs(rotInput) > 0.1 ? rotInput*0.5 : 0;
        return rotInput;
    }
    /**
     * Inner class containing controller bindings
     */
    private static class Bindings {
        /** Button to fire ball from shooter */
        static final Button SHOOTER_FIRE_BUTTON = XboxController.Button.kB;
        /** Button to re-zero gyro */
        static final Button ZERO_GYRO = XboxController.Button.kBack;
        /** Button to intake ball manually*/
        static final Button INTAKE_BUTTON = XboxController.Button.kRightBumper;
        /** Button to pick up balls automatically */
        static final Button BALL_TRACKING_BUTTON = XboxController.Button.kLeftBumper;
        /** Button to drive at reduced speed */
        static final Button SLOW_DRIVE_BUTTON = XboxController.Button.kRightBumper;

        //Climber Button
        static final Button CLIMBER_BUTTON = XboxController.Button.kLeftBumper;

        //Operator adjustment buttons
        static final Button OPERATOREVALUATION_UNDERSHOOT = XboxController.Button.kX;
        static final Button OPERATOREVALUATION_OVERSHOOT = XboxController.Button.kB;
        static final Button OPERATOREVALUATION_SHOTHIT = XboxController.Button.kA;
        static final Button OPERATOREVALUATION_BOUNCEOUT = XboxController.Button.kY;

        static final Button RELEASE_BALL_BUTTON = XboxController.Button.kBack;
        static final Button MANUAL_CLIMB_BUTTON = XboxController.Button.kStart;
    }

    /** Port for controller used by driver */
    private static final int DRIVER_CONTROLLER_PORT = 0;
    /** Port for controller used by operator */
    private static final int OPERATOR_CONTROLLER_PORT = 1;

    /** Controller used by driver, mapped to {@link #DRIVER_CONTROLLER_PORT} */
    public static final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
    /** Controller used by driver, mapped to {@link #OPERATOR_CONTROLLER_PORT} */
    public static final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

    /** button for shoot command. Mapped to {@link Bindings#SHOOTER_FIRE_BUTTON} */
    public static final JoystickButton shootButton = new JoystickButton(driverController, Bindings.SHOOTER_FIRE_BUTTON.value);
    /** Example button. Mapped to {@link Bindings#ZERO_GYRO} */
    public static final JoystickButton zeroButton = new JoystickButton(driverController, Bindings.ZERO_GYRO.value);
    /** Button to deploy intake for 5 seconds. Mapped to {@link Bindings#INTAKE_BUTTON} */
    public static final JoystickButton intakeButton = new JoystickButton(operatorController, Bindings.INTAKE_BUTTON.value);
    /** Ball tracking button. Mapped to {@link Bindings#BALL_TRACKING_BUTTON} */
    public static final JoystickButton ballTrackingButton = new JoystickButton(driverController, Bindings.BALL_TRACKING_BUTTON.value);
    
    /** Drive reduced speed button. Mapped to {@link Bindings#SLOW_DRIVE_BUTTON} */
    public static final JoystickButton slowDriveButton = new JoystickButton(driverController, Bindings.SLOW_DRIVE_BUTTON.value);

    /** undershoot button. Mapped to {@link Bindings#OPERATOREVALUATION_UNDERSHOOT} */
    public static final JoystickButton undershootButton = new JoystickButton(operatorController, Bindings.OPERATOREVALUATION_UNDERSHOOT.value);
    /** overshoot button. Mapped to {@link Bindings#OPERATOREVALUATION_OVERSHOOT} */
    public static final JoystickButton overshootButton = new JoystickButton(operatorController, Bindings.OPERATOREVALUATION_OVERSHOOT.value);
    /** hit shot button. Mapped to {@link Bindings#OPERATOREVALUATION_SHOTHIT} */
    public static final JoystickButton shothitButton = new JoystickButton(operatorController, Bindings.OPERATOREVALUATION_SHOTHIT.value);
    /** bounceout button. Mapped to {@link Bindings#OPERATOREVALUATION_BOUNCEOUT} */
    public static final JoystickButton bounceoutButton = new JoystickButton(operatorController, Bindings.OPERATOREVALUATION_BOUNCEOUT.value);
    
    /** climb button. Mapped to {@link Bindings#CLIMBER_BUTTON} */
    public static final JoystickButton climbButton = new JoystickButton(operatorController, Bindings.CLIMBER_BUTTON.value);

    /** climb button. Mapped to {@link Bindings#MANUAL_CLIMB_BUTTON} */
    public static final JoystickButton manualClimbButton = new JoystickButton(operatorController, Bindings.MANUAL_CLIMB_BUTTON.value);
    
    /** climb button. Mapped to {@link Bindings#CLIMBER_BUTTON} */
    public static final JoystickButton releaseBallButton = new JoystickButton(operatorController, Bindings.RELEASE_BALL_BUTTON.value);



}
 
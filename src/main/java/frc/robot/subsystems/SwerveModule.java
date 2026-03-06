package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Swerve module class designed to imitate CTRE's library, but for older MK2 hardware
 */
public class SwerveModule {
    
    SwerveModuleConstants constants;

    CANSparkMax driveSparkmax;
    CANSparkMax steerSparkmax;
    AnalogPotentiometer encoder;

    public SwerveModule(SwerveModuleConstants constants, String canbusname){
        this.constants = constants;

        driveSparkmax = new CANSparkMax(constants.DriveMotorId, MotorType.kBrushless);
        steerSparkmax = new CANSparkMax(constants.SteerMotorId, MotorType.kBrushless);
        encoder = new AnalogPotentiometer(new AnalogInput(constants.CANcoderId), 360, 360 - constants.CANcoderOffset);

        SmartDashboard.putData(encoder);

        // Configure drive motor
        double driveFactor = constants.DriveMotorGearRatio * constants.WheelRadius * 2 * Math.PI;
        driveSparkmax.getEncoder().setPositionConversionFactor(driveFactor);
        driveSparkmax.getEncoder().setVelocityConversionFactor(driveFactor);
        driveSparkmax.setInverted(constants.DriveMotorInverted);

        // Configure steer motor
        steerSparkmax.setIdleMode(IdleMode.kBrake);
        steerSparkmax.getEncoder().setPositionConversionFactor(constants.SteerMotorGearRatio);
        steerSparkmax.getEncoder().setVelocityConversionFactor(constants.SteerMotorGearRatio);
        steerSparkmax.getEncoder().setPosition(1 - (encoder.get() / 360));
        SparkPIDController pidController = steerSparkmax.getPIDController();
        pidController.setP(constants.SteerMotorGains.kP);
        pidController.setI(constants.SteerMotorGains.kI);
        pidController.setD(constants.SteerMotorGains.kD);

    }

    public void configNeutralMode(NeutralModeValue value){
        if (value == NeutralModeValue.Brake)
            driveSparkmax.setIdleMode(IdleMode.kBrake);
        else if (value == NeutralModeValue.Coast)
            driveSparkmax.setIdleMode(IdleMode.kCoast);
    }

    public void getPosition(boolean unkown){ }

    public SwerveModuleState getCurrentState(){
        SwerveModuleState state = new SwerveModuleState();
        state.speedMetersPerSecond = driveSparkmax.getEncoder().getVelocity();
        state.angle = Rotation2d.fromRotations(steerSparkmax.getEncoder().getPosition());
        return state;
    }

    public SwerveModulePosition getCachedPosition(){
        return new SwerveModulePosition(
            driveSparkmax.getEncoder().getPosition(),
            getCurrentState().angle
        );
    }

    public void apply(SwerveModuleState state, DriveRequestType type){
        if (type == DriveRequestType.OpenLoopVoltage){
            driveSparkmax.set(state.speedMetersPerSecond);
            steerSparkmax.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition);
        }
    }
}

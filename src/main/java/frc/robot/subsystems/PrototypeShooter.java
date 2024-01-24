package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class PrototypeShooter extends SubsystemBase{
    private TalonFX motorFalcon1 = new TalonFX(RobotMap.CANID.PROTOTYPE_SHOOTER1);
    private TalonFX motorFalcon2 = new TalonFX(RobotMap.CANID.PROTOTYPE_SHOOTER2);

    public PrototypeShooter() {

    }
    /** Takes a value -1 and 1 if at 0 doesn't run
     * Sets motor speed for shooter
     */
    public void setShooterSpeed(double Speed){
        motorFalcon1.set(Speed);
        motorFalcon2.set(Speed);
    }

    /**
     * Stops motor for shooter
     */
    public void stopShooter(){
        motorFalcon1.set(0);
        motorFalcon2.set(0);
    }

    
}
package frc.robot.subsystems;

public class PrototypeShooter extends SubsystemBase{
    private TalonFX motorFalcon1 = new TalonFX(RobotMap.CANID.PROTOTYPE_SHOOTER1);
    private TalonFX motorFalcon2 = new TalonFX(RobotMap.CANID.PROTOTYPE_SHOOTER2);

    public ProtostypeShooter() {

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
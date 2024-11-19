package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;

public class DemoShooterCommand extends Command {

    // IF YOU WANT TO CHANGE SHOOTER SPEED, CHANGE THE SELECTION HERE
    private static final SpeedConfig SELECTED_SPEED = SpeedConfig.DISABLED;

    public DemoShooterCommand(){
        addRequirements(RobotContainer.m_shooter);
    }

    @Override
    public void initialize(){
        RobotContainer.m_shooter.setShooterSpeed(SELECTED_SPEED.main);
        RobotContainer.m_shooter.setTopShooterSpeed(SELECTED_SPEED.top);
    }

    @Override
    public void end(boolean isInterrupted){
        if(SELECTED_SPEED.idle == 0){
            RobotContainer.m_shooter.coastToIdle();
        } else {
            RobotContainer.m_shooter.setShooterSpeed(SELECTED_SPEED.main * SELECTED_SPEED.idle);
            RobotContainer.m_shooter.setTopShooterSpeed(SELECTED_SPEED.top * SELECTED_SPEED.idle);
        }
    }

    private static class SpeedConfig{
        public final double main;
        public final double top;
        public final double idle;
        public SpeedConfig(double main, double top, double idle){
            this.main = main;
            this.top = top;
            this.idle = idle;
        }

        // Add more speed configurations here
        static final SpeedConfig DISABLED = new SpeedConfig(0, 0, 0);
        static final SpeedConfig BASKETBALL = new SpeedConfig(-3000, 3000, 2000);
    }
}

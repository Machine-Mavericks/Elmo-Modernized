// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDBlinkin.LED_PATTERN;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Command controls LEDs to display required information to driver */
public class LEDCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  /**
   * Creates a new LEDCommand.
   */
  public LEDCommand() {
    //addRequirements(RobotContainer.LEDStrip);
    addRequirements(RobotContainer.LEDStrip);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  int counter = 0;
  @Override
  public void execute() {
    
    counter +=1;
    if (counter>25)
      counter=0;


    
    
    // ball present
    if (RobotContainer.ballTargeting.IsBall())
    {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red)
          RobotContainer.LEDStrip.setPattern(LED_PATTERN.REDBALL);
        else
          RobotContainer.LEDStrip.setPattern(LED_PATTERN.BLUEBALL);
      }else{
        RobotContainer.LEDStrip.setPattern(LED_PATTERN.OFF);
      }
    }
    
    // // 20 seconds remaining warning
    // if (DriverStation.getMatchTime() <= 60.0 && counter >=0 && counter<=12)
    // {
    //   RobotContainer.LEDStrip.setPattern(LED_PATTERN.DISCO);
    // }  

    // hub present
    else if (RobotContainer.hubTargeting.IsTarget() && counter >12 && counter <25)
    {
      RobotContainer.LEDStrip.setPattern(LED_PATTERN.HUB);
    }
    // // low battery voltage warning
    // else if (RobotContainer.panel.getVoltage() <11.0)
    // {
    //    RobotContainer.LEDStrip.setPattern(LED_PATTERN.LOWBATTERY);
    // }

    
    // // should we turn off?
    // if ( !(RobotContainer.panel.getVoltage() <11.0 ) &&
    //      !(RobotContainer.ballTargeting.IsBall()) &&
    //      !(RobotContainer.hubTargeting.isTargetPresent()))
    // {
    //   RobotContainer.LEDStrip.setPattern(LED_PATTERN.OFF);
    // }
    
    // // counter used to create blink rate of ~2.5Hz
    // counter++;
    // if (counter>=20)
    //   counter=0;
    
    // // use counter to interleave blinks of LEDs as req'd
    // if (counter==0)
    // {
    //   if (RobotContainer.hubTargeting.isTargetPresent()){
    //     if (RobotContainer.hubTargeting.IsTarget() && RobotContainer.hubTargeting.ReadyToShoot()){RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 255, 0);} // Distance and rotation are right, ready to shoot!
    //     else{RobotContainer.LEDStrip.SetEntireStripColorRGB(255, 0, 255);} // Hub in sight, but not ready to shoot.
    //   }
    //   else{RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0);} // LEDs are blank if hub cannot be found.
    // }
    // else if (counter==10)
    // {
    //   if (RobotContainer.ballTargeting.IsBall())
    //   {
    //     if (DriverStation.getAlliance() == Alliance.Red){RobotContainer.LEDStrip.SetEntireStripColorRGB(255, 0, 0);} // Sees red ball.
    //     else{RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 255);} // Sees blue ball.
    //   }
    //   else{RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0);} // Leds are blank if ball cannot be found .
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0); // Turn off LEDs when command ends.
    RobotContainer.LEDStrip.setPattern(LED_PATTERN.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** 
 * A convenient interface for using shuffleboard, supports disabling shuffleboard for the base class, or setting a custom update rate.
 * Managed through the static RobotShuffleboardManager class
 */
public interface ShuffleUser {
    public void initializeShuffleboard();
    
    public void updateShuffleboard();
} 

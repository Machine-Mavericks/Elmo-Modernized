// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Definitely one of the worst things in the codebase
// Why aren't status signals public?
package com.ctre.phoenix6.mechanisms.swerve;

import com.ctre.phoenix6.BaseStatusSignal;

/* Horiffic. Disgusting hack. */
public class PhoenixUnsafeAccess {
    public static BaseStatusSignal[] getSwerveSignals(SwerveModule module){
        return module.getSignals();
    }
}

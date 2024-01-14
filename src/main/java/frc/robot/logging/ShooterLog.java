// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.ArrayList;

import frc.robot.commands.ShotEvaluationCommand.ShotType;

/** Add your docs here. */
public class ShooterLog {

    private static ShooterLog _instance;

    public ArrayList<ShotData> pastShots = new ArrayList<ShotData>();

    private ShooterLog(){}

    public static ShooterLog instance(){
        if(_instance == null){
            _instance = new ShooterLog();
        }
        return _instance;
    }

    public void logShot(ShotData sd){
        pastShots.add(sd);
    }

    public static class ShotData {
        public ShotType type;
        public double x;
        public double y;
        public double rotation;

        public ShotData(ShotType type, double x, double y, double rotation){
            this.type = type;
            this.x = x;
            this.y = y;
            this.rotation = rotation;
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

import frc.robot.Robot;

/** Add your docs here. */
public class RobotShuffleboardManager {
    public static ArrayList<ShuffleUser> m_shuffleUserList = new ArrayList<>();
    public static HashMap<Double, ShuffleboardUpdateRunnable> m_updateTasks = new HashMap<>();

    private static Robot m_robot;
    private static boolean m_setupComplete = false;

    private static final double DefaultUpdatesPerSecond = 20;

    private static class ShuffleboardUpdateRunnable implements Runnable{
        ArrayList<ShuffleUser> updateList = new ArrayList<>();
        public void addUser(ShuffleUser shuffleUser){
            updateList.add(shuffleUser);
        }

        public void run(){
            for (int i = 0; i < updateList.size(); ++i){
                updateList.get(i).updateShuffleboard();
            }
        }
    }

    public static void init(Robot robot){
        m_setupComplete = true;
        m_robot = robot;
    }

    public static void RegisterShuffleUser(ShuffleUser shuffleUser){
        RegisterShuffleUser(shuffleUser, true, DefaultUpdatesPerSecond);
    }

    public static void RegisterShuffleUser(ShuffleUser shuffleUser, boolean enabled){
        RegisterShuffleUser(shuffleUser, enabled, DefaultUpdatesPerSecond);
    }

    public static void RegisterShuffleUser(ShuffleUser shuffleUser, boolean enabled, double updatesPerSecond){
        if (!m_setupComplete) {
            System.out.println("Shuffleboard Manager not setup yet, setup must be called first in robot initialization");
            return;
        }

        if (!enabled) {
            return;
        }

        // Avoid adding same class twice
        if (m_shuffleUserList.contains(shuffleUser)) {
            System.out.println("Cannot register same class twice. Class was: " + shuffleUser.getClass().getName());
            return;
        }

        m_shuffleUserList.add(shuffleUser);
        shuffleUser.initializeShuffleboard();
        SetupUpdateTask(shuffleUser, updatesPerSecond);
    }

    private static void SetupUpdateTask(ShuffleUser shuffleUser, double updatesPerSecond){
        ShuffleboardUpdateRunnable runnable;
        if (m_updateTasks.containsKey(updatesPerSecond)) {
            runnable = m_updateTasks.get(updatesPerSecond);
        }else{
            runnable = new ShuffleboardUpdateRunnable();
            m_updateTasks.put(updatesPerSecond, runnable);
            m_robot.addPeriodic(runnable, 1 / updatesPerSecond);
        }
        runnable.addUser(shuffleUser);
    }
}

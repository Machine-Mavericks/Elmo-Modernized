// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

import frc.robot.Robot;

/** 
 * Helper for keeping shufflebaord code clean.
 * Use RegisterShuffleUser() to setup parameters how often to update shuffleboard.
 * RegisterShuffleUser() has a few overloads, namely letting you easily disable this tab while keeping all the relevant code around.
 * As a rule of thumb to keep shuffleboard clean, try to disable any subsystems that aren't being actively debugged.
 * Persistent shuffleboard data should be managed through shuffleboardOI.
 * To init this class call init() with a reference to the main robot class.
 * To make a subsystem use this class implement the ShuffleUser interface.
 */
public class SubsystemShuffleboardManager {
    public static ArrayList<ShuffleUser> m_shuffleUserList = new ArrayList<>();
    public static HashMap<Double, ShuffleboardUpdateRunnable> m_updateTasks = new HashMap<>();

    private static Robot m_robot;
    private static boolean m_setupComplete = false;

    // Equal to the default periodic update rate
    private static final double DefaultUpdatesPerSecond = 20;
    private static final double DefaultUpdateOffsetSeconds = 0.01;

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
            System.out.println("Shuffleboard Manager not setup yet, init must be called first in robot initialization");
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
            m_robot.addPeriodic(runnable, 1 / updatesPerSecond, DefaultUpdateOffsetSeconds);
        }
        runnable.addUser(shuffleUser);
    }
}

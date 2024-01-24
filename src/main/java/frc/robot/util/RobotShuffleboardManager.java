// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

/** Add your docs here. */
public class RobotShuffleboardManager {
    public static ArrayList<InternalShuffleUserData> shuffleUserList = new ArrayList<>();
    private static final double DefaultUpdatesPerSecond = 20;

    private static class InternalShuffleUserData {
        public InternalShuffleUserData(ShuffleHelper shuffleUser, boolean enabled, double updatesPerSecond){
            this.shuffleUser = shuffleUser;
            this.enabled = enabled;
            this.updatesPerSecond = updatesPerSecond;
        }

        public final ShuffleHelper shuffleUser;

        public final double updatesPerSecond;
        public final boolean enabled;

    }

    public RobotShuffleboardManager(){
        
    }

    public static void RegisterShuffleUser(ShuffleHelper shuffleUser){
        RegisterShuffleUser(shuffleUser, true, DefaultUpdatesPerSecond);
    }

    public static void RegisterShuffleUser(ShuffleHelper shuffleUser, boolean enabled){
        RegisterShuffleUser(shuffleUser, enabled, DefaultUpdatesPerSecond);
    }

    public static void RegisterShuffleUser(ShuffleHelper shuffleUser, boolean enabled, double updatesPerSecond){
        shuffleUserList.add(new InternalShuffleUserData(shuffleUser, enabled, updatesPerSecond));
    }
}

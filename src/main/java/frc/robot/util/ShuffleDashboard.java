package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// DO NOT USE
// TESTING PURPOSES
public class ShuffleDashboard {
    private ShuffleboardTab Tab;
    private String Name;
    private boolean DebugEnabled;

    HashMap<String, GenericEntry> DebugEntryList = new HashMap<String, GenericEntry>();

    public ShuffleDashboard(String name, boolean debugEnabled){
        Name = name;
        DebugEnabled = debugEnabled;
    }

    public void putDebugString(String key, Object value){
        if (!DebugEnabled) {
            return;
        }

        getDebugEntry(key, "").setValue(value);
    }

    /**
     * Returns the entry with a specified name, if it doesn't exist, create it
     * @return an entry for the desired key
     */
    private GenericEntry getDebugEntry(String key, Object defaultValue){
        GenericEntry entry = DebugEntryList.get(key);
        if (entry == null) {
            entry = getTab().add(key, defaultValue).getEntry();
            DebugEntryList.put(key, entry);
        }
        return entry;
    }

    /**
     * Returns the tab for this ShuffleDashbaord. If it doesn't exist yet, it creates it. Useful for standard persistent shuffleboard stuff. 
     * Warning, only call this function if you want the tab to exist. ShuffleDashboards with disabled debugging won't create a tab to keep shuffleboard clean
     * @return
     */
    public ShuffleboardTab getTab(){
        if (Tab == null) {
            Tab = Shuffleboard.getTab(Name);
        }
        return Tab;
    }

}
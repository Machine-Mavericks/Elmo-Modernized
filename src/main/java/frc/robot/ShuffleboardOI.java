// Shuffleboard
//
// Initializes and updates shuffleboard
// This module contains code for making and maintaining main shuffleboard page
// Other pages made by the individual subsystems as req'd

package frc.robot;


import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericEntry;


/** Contains shuffleboard setup for generic main page not belonging to any subsubsystem
 * Typically used to have robot-level settings and command functions */
public class ShuffleboardOI extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private GenericEntry m_delayTime;
    private SendableChooser<Integer> m_autonomousPath;

    // other controls on main page
    private GenericEntry m_timeLeft;
    public Integer m_selectedPath;

    /** Initializes the Shuffleboard
     * Creates the subsystem pages */
    public ShuffleboardOI() {

        // add autonomous commands to shuffleboard
        initializeMainShuffleboardPage();
    }

    /** Update Shuffleboard Pages. This method will be called once per scheduler run
     * (=50Hz) */
    @Override
    public void periodic() {

        // update main page
        // update remaining time in match (rounded to nearest second)
        m_selectedPath = (Integer)m_autonomousPath.getSelected();
        m_timeLeft.setDouble(Math.round(Timer.getMatchTime()));
    }

    
    /** returns delay for autonomous routines */
    public double getAutoDelay()
    {
        return m_delayTime.getDouble(0.0);
    }

    // -------------------- Shuffboard Methods --------------------


    /** Create main shuffleboard page
     * Typically contains autonomous commands and other top-level robot controls*/
    private void initializeMainShuffleboardPage() {

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        m_autonomousPath = new SendableChooser<Integer>();

        // add autonomous commands to page -
        m_autonomousPath.addOption("Anywhere Two-ball",0);
        m_autonomousPath.addOption("Five-ball",1);
        m_autonomousPath.addOption("Emerg 4-ball", 2);
        m_autonomousPath.setDefaultOption("Anywhere Two-ball", 0);
        //m_autonomousPath.addOption("Two-ball auto",0);
        //m_autonomousPath.addOption("Three-ball auto",1);
        //m_autonomousPath.addOption("Anywhere Two-ball",2);
        //m_autonomousPath.addOption("Five-ball",3);
        //m_autonomousPath.setDefaultOption("One-ball auto", 3);
        tab.add("Preround Paths", m_autonomousPath).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
        m_delayTime = tab.add("Auto Delay Time", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1).withSize(1, 1).withProperties(Map.of("min", 0, "max", 10)).getEntry();

              
        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab.getLayout("Timer", BuiltInLayouts.kList);
        l1.withPosition(0, 2);
        l1.withSize(1, 2);
        m_timeLeft = l1.add("TimeLeft", 0.0).getEntry();
    }

    // returns position of autonomous commands on shuffleboard
    // typically called by Robot AutonomousInit to select auto path to be followed
    // returns true if selected, false if not
    // TODO <to be revised for 2022 robot>



} // end class ShuffleboardOI

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.paths.DoNothing;
import frc.robot.autonomous.paths.Baseline;
import frc.robot.autonomous.paths.HeadOn;

/**
 * Adds and selects autonomous paths by a SendableChooser.
 * @author hrl
 */
public class AutonHelper {
    private static AutonHelper instance;

    private SendableChooser<AutonPath> chooser = new SendableChooser<>();

    private AutonHelper() {
        this.addPath("Nothing", new DoNothing());
        this.addPath("Baseline", new Baseline());
        this.addPath("Head-On", new HeadOn());

        chooser.setDefaultOption("Head-On", new HeadOn());
    }

    /**
     * Returns a singleton instance of the auton helper.
     */
    public static AutonHelper getInstance() {
        if (instance == null) {
            return new AutonHelper();
        }

        return instance;
    }

    /**
     * Adds a path to the auton helper.
     * @param name the name to assign the autonomous path
     * @param func the autonomous path itself
     */
    public void addPath(String name, AutonPath func) {
        chooser.addOption(name, func);
    }

    /**
     * Returns the current autonomous path's key.
     */
    public AutonPath getCurrentPath() {
        return chooser.getSelected();
    }

    /**
     * Runs whatever path is selected on the sendable chooser.
     */
    public void runSelectedPath() {
        this.getCurrentPath().run();
    }

    /**
     * Resets the currently selected path to its initial state.
     */
    public void resetSelectedPath() {
        this.getCurrentPath().reset();
    }

    /**
     * Puts the SendableChooser to the SmartDashboard.
     */
    public void initSD() {
        SmartDashboard.putData(chooser);
    }
}

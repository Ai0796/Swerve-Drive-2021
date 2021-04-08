/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

/**
 * Stores constants for explicit usage in autonomous (e.g. distance). Should be
 * structured in a fashion where static classes are created for each path.
 * All units of displacement are in US Imperial Inches unless stated otherwise.
 * All units of power are from -1 to 1 unless stated otherwise.
 * All angles are in degrees unless stated otherwise.
 * @author common
 */
public class AutonMap {
    public static class Baseline {
        // the distance to drive from the baseline
        public static final double DISTANCE = 10.0;
        // the power to start driving with
        public static final double START_POWER = 0.5;
        // the power to end driving with
        public static final double END_POWER = 0;
    }

    public static class HeadOn {
        // the distance to drive (backwards) from the baseline
        // NOTE: since IPT is 1 (@Bloomington) this is actually encoder ticks
        public static final double DISTANCE = 10.0;
        // the power to start driving with
        public static final double START_POWER = 0.5;
        // the power to end driving with
        public static final double END_POWER = 0;

        // the backup (not PID) dead-reckoned power to run the shooter at
        public static final double BACKUP_SHOOTER_POWER = 0.73; // TODO: this should not be used
        // the power to run the shooter at
        public static final double SHOOTER_RPM = 4000; // TODO: tune
    }
    public static class Test{
        public static final double DISTANCE = 100.0;
        public static final double START_POWER = 1;
        public static final double END_POWER = 0;
    }
}

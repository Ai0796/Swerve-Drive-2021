package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonPath;

/**
 * Does nothing.
 * @author Who cares?
 */
public class DoNothing extends AutonPath {
    private final double DONT_SPEED = 0;

    @Override
    public void run() {
        dont();
    }

    private void dont() {
        Robot.drivetrain.setSpeed(DONT_SPEED, DONT_SPEED);
        System.out.println("!!!! DEPLOYING EVASIVE MANEUVERS !!!!");
    }

    @Override
    public void reset() {
        // NOTE: intentionally a stub
    }

    @Override
    public String toString() {
        return "Nothing";
    }
}

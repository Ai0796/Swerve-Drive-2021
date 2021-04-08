package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.AutonPath;

/** 
 * Gets the robot to the baseline.
 * @author jk, hrl
 */
public class Baseline extends AutonPath {
    private static double currentDistance = 0;
    private BaselineState currentState = BaselineState.ToBaseline;

    private enum BaselineState {
        ToBaseline {
            @Override
            public BaselineState nextState() {
                if (currentDistance < AutonMap.Baseline.DISTANCE) {
                    return this;
                }
                return Done;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.Baseline.DISTANCE, AutonMap.Baseline.START_POWER,
                        AutonMap.Baseline.END_POWER);
            }
        },
        Done {
            @Override
            public BaselineState nextState() {
                return this;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
            }
        };

        public abstract BaselineState nextState();
        public abstract void run();
    }

    /**
     * Runs the autonomous path.
     */
    @Override
    public void run() {
        currentDistance = Robot.drivetrain.getCurrentDistance();

        this.currentState.run();
        this.currentState = this.currentState.nextState();
    }

    @Override
    public void reset() {
        this.currentState = BaselineState.ToBaseline;
    }

    @Override
    public String toString() {
        return "Baseline";
    }
}

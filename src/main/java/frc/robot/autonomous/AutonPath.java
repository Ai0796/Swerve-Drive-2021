package frc.robot.autonomous;

/**
 * The implementation of an autonomous path.
 * Basically only needed for AutonHelper. Don't think about this too much.
 * @author hrl
 */
public abstract class AutonPath {
    public abstract void run();
    public abstract void reset();
    @Override
    public abstract String toString();
}

package frc.robot.autonomous;

public class MotionProfileGenerator {

    private static final double maxVelocity = 0; //Feet per second Currently 0 because untested
    private static final double maxAcceleration = 0; //Feet per second^2 Currently 0 because untested

    /*You have no idea how much of this code is based of 254
    I literally just have 254's code open while coding this
    MotionStates contain a velocity and position
    */
    public static MotionProfile generateProfile(MotionStateEnd end,
                                                MotionState start)
    {
        double deltaPos = end.pos() - start.pos();
        double deltaVelInitial = maxVelocity - start.vel(); //Starting Velocity difference
        double deltaVelFinal = end.vel() - maxVelocity; //Ending Velo
        
        double accel_time = Math.abs(deltaVelInitial/maxAcceleration); // Add Jerk (change in Acceleration) later
        double decel_time = Math.abs(deltaVelFinal/maxAcceleration);

        double accel_distance = kinematicDistance(start.vel(), maxAcceleration, accel_time);
        double decel_distance = kinematicDistance(maxVelocity, -maxAcceleration, decel_time);
        
        double cruise_distance = deltaPos - accel_distance - decel_distance;
        double cruise_time = cruise_distance/maxVelocity;
    }

    // d = vt + 1/2at^2
    public static double kinematicDistance(double v, double a, double t){
        return((v * t) + (a * (Math.pow(t, 2)/2))); 
    }

    // vf^2 = vi^2 + 2ad
    public static double kinematicFinalVelUsingDistance(double vi, double a, double d){
        return Math.sqrt(Math.pow(vi, 2) + (2 * a * d)); 
    }

    // vf = vi + at
    public static double kinematicFinalVelUsingTime(double vi, double a, double t){
        return vi + a * t; 
    }
}

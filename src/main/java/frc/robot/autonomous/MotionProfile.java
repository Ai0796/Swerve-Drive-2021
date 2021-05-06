package frc.robot.autonomous;

public class MotionProfile {

    private double accelTime, accelDistance, cruiseTime, cruiseDistance, decelTime, decelDistance;
    
    public void MotionProfile(double accel_time, double accel_distance, double cruise_time, double cruise_distance, double decel_time, double decel_distance){

        accelTime = accel_time;
        accelDistance = accel_distance;
        cruiseTime = cruise_time;
        cruiseDistance = cruise_distance;
        decelTime = decel_time;
        decelDistance = decel_distance;

    }

    public void run(){
        
    }
}

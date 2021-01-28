package frc.robot;


public class CircleGeometry {
    
    /*
    Class is heavily Inspired on 254's code Rotation2d
    */

    protected static double radius_ = Double.NaN;
    protected static double radians_ = Double.NaN;
    protected static double sin_angle_ = Double.NaN;
    protected static double cos_angle_ = Double.NaN;


    //Constants
    private double Pi = Math.PI;
    private static double k2Pi = Math.PI * 2;
    private double kEpsilon = 1e-12;

    public CircleGeometry(double radius, double radians) {
        // Takes Radius and radians, computes sin and cos
        radius_ = radius;
        radians_ = radians;
        sin_angle_ = Math.sin(radians_);
        cos_angle_ = Math.cos(radians_);
    }

    public double sin() {
        verifyTrig();
        return sin_angle_;
    }

    public double cos() {
        verifyTrig();
        return cos_angle_;
    }

    public double tan(){
        verifyTrig();
        if(Math.abs(cos_angle_) < kEpsilon){
            if(sin_angle_ >= 0){
                return Double.POSITIVE_INFINITY;
            }else{
                return Double.NEGATIVE_INFINITY;
            }
        }else{
            return sin_angle_/cos_angle_;
        }
    }

    public double asin(){
        if(Math.abs(sin_angle_) < kEpsilon){
            return Double.POSITIVE_INFINITY;
        }else{
            return 1/sin_angle_;
        }
    }

    public double acos(){
        if(Math.abs(cos_angle_) < kEpsilon){
            return Double.POSITIVE_INFINITY;
        }else{
            return 1/cos_angle_;
        }
    }

    public double atan(){
        if(Math.abs(sin_angle_) < kEpsilon){
            if(cos_angle_ >= 0){
                return Double.POSITIVE_INFINITY;
            }else{
                return Double.NEGATIVE_INFINITY;
            }
        }else{
            return cos_angle_/sin_angle_;
        }
    }

    public double getRadians(){
        verifyRadians();
        return(radians_);
    }

    public double getDegrees(){
        verifyRadians();
        return Math.toDegrees(radians_);
    }

    public double getRadius(){
        verifyRadius();
        return radius_;
    }

    public void setRadians(double radian){
        radians_ = WrapRadians(radian);
    }

    public double WrapRadians(double radians) {
        // Wraps radians to be between -pi and pi
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    private static void verifyTrig(){
        if(!hasTrig()){
            if (!hasRadians()){
                System.err.println("Error: No Radians");
            }else{
                sin_angle_ = Math.sin(radians_);
                cos_angle_ = Math.cos(radians_);
            }
        }
    }

    private void verifyRadians(){
        if(!hasRadians()){
            if (!hasTrig()){
                System.err.println("Error: Lack of Sin angle or Cos angle");
            }else{
                radians_ = atan();
                if(sin_angle_ < 0){
                    radians_ -= Pi;
                }
            }
        }
    }

    private void verifyRadius(){
        //Can't calculate radius with any of our numbers, so assume it equals 1
        if(!hasRadius()){
            radius_ = 1;
        }
    }

    //Please rename this method
    public boolean checkFastestTurn(double angle){
        return(Math.abs(angle - radians_) < Math.abs(-angle - radians_));
    }

    private static boolean hasRadians(){
        return !Double.isNaN(radians_);
    }

    private static boolean hasTrig(){
        return (!Double.isNaN(sin_angle_) || !Double.isNaN(cos_angle_));
    }

    private static boolean hasRadius(){
        return !Double.isNaN(radius_);
    }
}

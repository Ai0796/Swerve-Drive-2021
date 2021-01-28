package frc.robot;

import com.fasterxml.jackson.core.sym.Name;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDriveModule {
    
    CircleGeometry EncoderGear;
    CANSparkMax DriveMotor;
    CANSparkMax EncoderMotor;
    double xDistance;
    double yDistance;
    AnalogEncoder swerveEncoder;
    CANEncoder driveEncoder;
    String Name;

    double targetWheelSpeed = 0;
    double targetEncoderLocation = 0;

    double kEpsilon = 1e-13;


    public SwerveDriveModule (CircleGeometry Encoder_Gear, CANSparkMax Drive_Motor, CANSparkMax Encoder_Motor, double x_Distance, double y_Distance, AnalogEncoder Encoder, String name){
        
        EncoderGear = Encoder_Gear;
        DriveMotor = Drive_Motor;
        EncoderMotor = Encoder_Motor;
        xDistance = x_Distance;
        yDistance = y_Distance;
        swerveEncoder = Encoder;
        swerveEncoder.setDistancePerRotation(RobotMap.SwerveDrive.ENCODER_GEAR_RATIO);
        driveEncoder = Drive_Motor.getEncoder();
        Name = name;
    }

    public void setVelocity(double x, double y, double r){
        targetWheelSpeed = wheelVectorCalculator(x, y, r);
        targetEncoderLocation = wheelAngleCalculator(x, y, r);
        if(checkFastestTurn(targetEncoderLocation)){
            targetWheelSpeed *= -1;
            targetEncoderLocation = EncoderGear.WrapRadians(targetEncoderLocation + Math.PI);
        }
    }

    //Checks if turning the other way and inversing speed would be faster
    public boolean checkFastestTurn(double angle){
        return Math.abs(getSwerveEncoderAngleRadians() - angle) > Math.abs(getSwerveEncoderAngleRadians() - EncoderGear.WrapRadians(angle + Math.PI));
    }

    public double wheelVectorCalculator(double x, double y, double r){
        double xVector;
        double yVector;
        double speedVector;
        xVector = x + (r * xDistance);
        yVector = y + (r * yDistance);
        speedVector = Math.sqrt(Math.pow(xVector, 2) + Math.pow(yVector, 2));

        return speedVector;
    }

    public double wheelAngleCalculator(double x, double y, double r){
        double xVector;
        double yVector;
        double angle;
        xVector = x + (r * xDistance);
        yVector = y + (r * yDistance);
        if(Math.abs(yVector) < kEpsilon){
            angle = 0;
        }else{
            angle = Math.atan(xVector/yVector);
        }
        return angle;
    }

    //The Encoder essentially works like a potentiometer
    //This function call is called repeatedly even with no changes to velocity
    public void goToVelocity(){

        //PID Needs testing
        double encoderSpeed = PID(targetEncoderLocation, 1, 0, 0);
        setDriveSpeed(targetWheelSpeed);
        setEncoderSpeed(encoderSpeed);
    }

    public void setEncoderSpeed(double speed){
        EncoderMotor.set(speed);
    }

    public void setDriveSpeed(double speed){
        DriveMotor.set(speed);
    }

    //The Encoder essentially works like a potentiometer
    public double PID(double targetDistance, double P, double I, double D){
        //PID Needs testing

        EncoderGear.setRadians(getSwerveEncoderAngleRadians());
        double distanceError = (targetDistance - EncoderGear.getRadians())/Math.PI;
        // double velocityError = distanceError/RobotMap.Common.UPDATE_PERIOD;
        return (distanceError * P);
    }

    public double getSwerveEncoderAngleDegrees(){
        return Math.toDegrees(getSwerveEncoderAngleRadians());
    }

    //Returns a number between -pi and pi
    public double getSwerveEncoderAngleRadians(){
        return ((swerveEncoder.getDistance()/RobotMap.SwerveDrive.MAX_ENCODER_VOLTAGE) - 0.5) * Math.PI;
    }

    public double getCurrentEncoderLocation(){
        return EncoderGear.getRadians();
    }

    public double getTargetEncoderLocation(){
        return targetEncoderLocation;
    }

    public double getCurrentSpeed(){
        return driveEncoder.getVelocity();
    }

    public double getTargetSpeed(){
        return targetWheelSpeed;
    }

    public void UpdateSD(){
        SmartDashboard.putNumber("Current Speed of " + Name, getCurrentSpeed());
        SmartDashboard.putNumber("Current Encoder Rotation of " + Name, getSwerveEncoderAngleDegrees());
        SmartDashboard.putNumber("Target Speed of " + Name, getTargetSpeed());
        SmartDashboard.putNumber("Target Angle of " + Name, Math.toDegrees(getTargetEncoderLocation()));
    }
}

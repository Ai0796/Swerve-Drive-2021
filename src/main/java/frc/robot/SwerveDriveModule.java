package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;

public class SwerveDriveModule {
    
    CircleGeometry EncoderGear;
    CANSparkMax DriveMotor;
    CANSparkMax EncoderMotor;
    double xDistance;
    double yDistance;
    Encoder swerveEncoder;
    CANEncoder driveEncoder;

    double targetWheelSpeed = 0;
    double targetEncoderLocation = 0;

    double previousEncoderLocation = 0;

    double kEpsilon = 1e-13;


    public SwerveDriveModule (CircleGeometry Encoder_Gear, CANSparkMax Drive_Motor, CANSparkMax Encoder_Motor, double x_Distance, double y_Distance, Encoder Encoder){
        
        EncoderGear = Encoder_Gear;
        DriveMotor = Drive_Motor;
        EncoderMotor = Encoder_Motor;
        xDistance = x_Distance;
        yDistance = y_Distance;
        swerveEncoder = Encoder;
        driveEncoder = Drive_Motor.getEncoder();
    }

    public void setVelocity(double x, double y, double r){
        targetWheelSpeed = wheelVectorCalculator(x, y, r);
        targetEncoderLocation = wheelAngleCalculator(x, y, r);
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
        if(yVector < kEpsilon){
            angle = 0;
        }else{
            angle = Math.atan(xVector/yVector);
        }
        return angle;
    }

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

    public double PID(double targetDistance, double P, double I, double D){
        //PID Needs testing

        EncoderGear.setRadians(swerveEncoder.getDistance());
        double distanceError = (targetDistance - EncoderGear.getRadians())/Math.PI;
        // double velocityError = distanceError/RobotMap.Common.UPDATE_PERIOD;
        return (distanceError * P);
    }

    public double getPosition(){
        return EncoderGear.getRadians();
    }
}

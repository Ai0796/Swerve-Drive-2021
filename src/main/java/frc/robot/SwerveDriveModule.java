package frc.robot;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


public class SwerveDriveModule {
    
    CircleGeometry EncoderGear;
    CANSparkMax DriveMotor;
    CANSparkMax EncoderMotor;
    double xDistance;
    double yDistance;
    AnalogEncoder swerveEncoder;
    CANEncoder driveEncoder;
    String Name;

    boolean usingMotorEncoder;

    private final PIDController drivePIDController = new PIDController(
        RobotMap.SwerveDrive.DRIVE_P, 
        RobotMap.SwerveDrive.DRIVE_I, 
        RobotMap.SwerveDrive.DRIVE_D);

    private final ProfiledPIDController swervePIDController =
        new ProfiledPIDController(
            RobotMap.SwerveDrive.SWERVE_P,
            RobotMap.SwerveDrive.SWERVE_I,
            RobotMap.SwerveDrive.SWERVE_D,
            new TrapezoidProfile.Constraints(
                RobotMap.SwerveDrive.MAX_SWERVE_ANGULAR_VELOCITY, RobotMap.SwerveDrive.MAX_SWERVE_ANGULAR_ACCELERATION));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_swerveFeedforward = new SimpleMotorFeedforward(1, 0.5);

    double targetWheelSpeed = 0;
    double targetEncoderLocation = 0;

    double kEpsilon = 1e-13;


    public SwerveDriveModule (CircleGeometry Encoder_Gear, CANSparkMax Drive_Motor, 
                            CANSparkMax Encoder_Motor, double x_Distance, double y_Distance, 
                            AnalogEncoder Encoder, String name){
        
        //Set Variables
        EncoderGear = Encoder_Gear;
        DriveMotor = Drive_Motor;
        EncoderMotor = Encoder_Motor;
        xDistance = x_Distance;
        yDistance = y_Distance;
        swerveEncoder = Encoder;
        swerveEncoder.setDistancePerRotation(RobotMap.SwerveDrive.ENCODER_GEAR_RATIO);
        driveEncoder = Drive_Motor.getEncoder();
        Name = name;

        //Initialize Encoder Distances
        driveEncoder.setPositionConversionFactor(2 * Math.PI * RobotMap.SwerveDrive.WHEEL_RADIUS / RobotMap.SwerveDrive.ENCODER_RESOLUTION);

        //Limit Swerve PID Controller to (-pi, pi)
        swervePIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setVelocity(double x, double y, double r){
        targetWheelSpeed = wheelVectorCalculator(x, y, r);
        targetEncoderLocation = wheelAngleCalculator(x, y, r);
        if(checkFastestTurn(targetEncoderLocation)){
            targetWheelSpeed *= -1;
            targetEncoderLocation = EncoderGear.WrapRadians(targetEncoderLocation + Math.PI);
        }
    }

    /*Copied for WPILIB Example Swerve Drive
    */

    public void setDesiredVelocity(){
        double swerveVelocity;

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
        drivePIDController.calculate(driveEncoder.getVelocity(), targetWheelSpeed);

        final double driveFeedforward = m_driveFeedforward.calculate(targetWheelSpeed);

        // Calculate the turning motor output from the turning PID controller.
        swerveVelocity = (getSwerveEncoderAngleRadians()-EncoderGear.getRadians())/RobotMap.Common.UPDATE_PERIOD;
        EncoderGear.setRadians(getSwerveEncoderAngleRadians());
        final double turnOutput =
            swervePIDController.calculate(EncoderGear.getRadians(), targetEncoderLocation);

        final double swerveFeedforward =
            m_swerveFeedforward.calculate(swerveVelocity);

        setDriveSpeed(driveOutput + driveFeedforward);
        setEncoderSpeed(turnOutput + swerveFeedforward);
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

        // //PID Needs testing
        // double encoderSpeed = PID(targetEncoderLocation, 1, 0, 0);
        // setDriveSpeed(targetWheelSpeed);
        // setEncoderSpeed(encoderSpeed);
        setDesiredVelocity();
    }

    public void divideVelocity(double denominator){
        targetWheelSpeed /= denominator;
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
        SmartDashboard.putNumber("SparkMaxPower", DriveMotor.get());
        SmartDashboard.putNumber("Target Speed of " + Name, getTargetSpeed());
        SmartDashboard.putNumber("Target Angle of " + Name, Math.toDegrees(getTargetEncoderLocation()));
    }
}

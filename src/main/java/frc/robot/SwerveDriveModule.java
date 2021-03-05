package frc.robot;


import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.revrobotics.AlternateEncoderType;
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
    CANEncoder driveEncoder, altSwerveEncoder;
    double wheelRadius, wheelCircumference;
    String Name;
    Boolean usingAlternativeEncoder;
    double initialDistance = 884.199;

    double gear_ratio = 33.75;

    private final PIDController drivePIDController = new PIDController(
        RobotMap.SwerveDrive.DRIVE_P, 
        RobotMap.SwerveDrive.DRIVE_I, 
        RobotMap.SwerveDrive.DRIVE_D);

    private final ProfiledPIDController swervePIDController =
        new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                RobotMap.SwerveDrive.MAX_SWERVE_ANGULAR_VELOCITY / gear_ratio, RobotMap.SwerveDrive.MAX_SWERVE_ANGULAR_ACCELERATION/gear_ratio));

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
        driveEncoder.setVelocityConversionFactor(1/RobotMap.SwerveDrive.MAX_DRIVE_RPM);
        Name = name;

        //Initialize Encoder Distances
        driveEncoder.setPositionConversionFactor(2 * Math.PI * RobotMap.SwerveDrive.WHEEL_RADIUS / RobotMap.SwerveDrive.ENCODER_RESOLUTION);

        //Limit Swerve PID Controller to (-pi, pi)
        swervePIDController.enableContinuousInput(-Math.PI, Math.PI);

        //Set wheel Radius and Circumference
        wheelRadius = RobotMap.SwerveDrive.WHEEL_RADIUS;
        wheelCircumference = wheelRadius * 2 * Math.PI;

        usingAlternativeEncoder = false;
    }

    public SwerveDriveModule(CircleGeometry Encoder_Gear, CANSparkMax Drive_Motor, 
    CANSparkMax Encoder_Motor, double x_Distance, double y_Distance, String name){
        //Set Variables
        EncoderGear = Encoder_Gear;
        DriveMotor = Drive_Motor;
        EncoderMotor = Encoder_Motor;
        xDistance = x_Distance;
        yDistance = y_Distance;
        driveEncoder = Drive_Motor.getEncoder();
        driveEncoder.setVelocityConversionFactor(1/RobotMap.SwerveDrive.MAX_DRIVE_RPM);
        Name = name;

        //Initialize Encoder Distances
        driveEncoder.setPositionConversionFactor(2 * Math.PI * RobotMap.SwerveDrive.WHEEL_RADIUS / RobotMap.SwerveDrive.ENCODER_RESOLUTION);

        //Limit Swerve PID Controller to (-pi, pi)
        swervePIDController.enableContinuousInput(-Math.PI, Math.PI);

        //Set wheel Radius and Circumference
        wheelRadius = RobotMap.SwerveDrive.WHEEL_RADIUS;
        wheelCircumference = wheelRadius * 2 * Math.PI;

        //Uses Alternative Swerve Encoder
        altSwerveEncoder = Encoder_Motor.getEncoder();
        usingAlternativeEncoder = true;
    }

    public void setVelocity(double x, double y, double r){
        targetWheelSpeed = wheelVectorCalculator(x, y, r);
        targetEncoderLocation = wheelAngleCalculator(x, y, r);
        SmartDashboard.putBoolean("Reversed", false);
        if(checkFastestTurn(targetEncoderLocation)){
            SmartDashboard.putBoolean("Reversed", true);
            targetWheelSpeed = targetWheelSpeed * -1;
            targetEncoderLocation = EncoderGear.WrapRadians(targetEncoderLocation + Math.PI);
        }
        // setEncoderSpeed(r);
        // setDriveSpeed(y);
    }

    /*Copied from WPILIB Example Swerve Drive
    */

    public void setDesiredVelocity(){
        if(targetWheelSpeed != 0){
            double swerveVelocity;

            // Calculate the drive output from the drive PID controller.
            double driveOutput =
            drivePIDController.calculate(driveEncoder.getVelocity(), targetWheelSpeed);

            double driveFeedforward = m_driveFeedforward.calculate(targetWheelSpeed);

            // Calculate the turning motor output from the turning PID controller.
            swerveVelocity = (getSwerveEncoderAngleRadians()-EncoderGear.getRadians())/RobotMap.Common.UPDATE_PERIOD;

            EncoderGear.setRadians(getSwerveEncoderAngleRadians());

            double turnOutput =
                swervePIDController.calculate(EncoderGear.getRadians(), targetEncoderLocation);

            double swerveFeedforward = 0;
            // double swerveFeedforward = m_swerveFeedforward.calculate(swerveVelocity);


            SmartDashboard.putNumber("PID", turnOutput);
            setDriveSpeed(driveOutput * 0.15);
            setEncoderSpeed((turnOutput + swerveFeedforward) * 0.3);
            // setDriveSpeed(0.75);
            // setEncoderSpeed(0.75);
        }
        else{
            setDriveSpeed(0);
            setEncoderSpeed(0);
        }
    }

    //Checks if turning the other way and inversing speed would be faster
    public boolean checkFastestTurn(double angle){
        // SmartDashboard.putNumber("Turn Method", getSwerveEncoderAngleRadians() - angle);
        // SmartDashboard.putNumber("Alt Turn Method", getSwerveEncoderAngleRadians() - EncoderGear.WrapRadians(angle + Math.PI));
        return angle < 0;
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
        if(Math.abs(xVector) < kEpsilon){
            angle = Math.PI/2;
            if(yVector < 0){
                angle *= -1;
            }
        }else{
            angle = Math.atan(yVector/xVector);
            if(angle > 0 && yVector < 0){
                angle -= Math.PI;
            }else if(angle < 0 && xVector < 0){
                angle += Math.PI;
            }
        }
        return angle;
    }

    //The Encoder essentially works like a potentiometer
    //This function call is called repeatedly even with no changes to velocity
    //Changed function name to make more sense, calls changed function due to lazyness
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

    public double getSwerveEncoderAngleDegrees(){
        return Math.toDegrees(getSwerveEncoderAngleRadians());
    }

    //Returns a number between -pi and pi
    public double getSwerveEncoderAngleRadians(){
        // SmartDashboard.putNumber("Encoder Value", ((swerveEncoder.getDistance()/RobotMap.SwerveDrive.MAX_ENCODER_VOLTAGE) - 0.5) * Math.PI);
        if(usingAlternativeEncoder){
            return EncoderGear.WrapRadians(altSwerveEncoder.getPosition() / gear_ratio * 2 * Math.PI);
        }
        return ((swerveEncoder.getDistance()/RobotMap.SwerveDrive.MAX_ENCODER_VOLTAGE) - 0.5) * Math.PI;
    }

    public double getCurrentEncoderLocation(){
        return EncoderGear.getRadians();
    }

    public double getTargetEncoderLocation(){
        return targetEncoderLocation;
    }

    public double getCurrentRPM(){
        return driveEncoder.getVelocity();
    }

    public double getCurrentSpeed(){
        return driveEncoder.getVelocity() * wheelCircumference;
    }

    public double getTargetSpeed(){
        return targetWheelSpeed;
    }

    public void UpdateSD(){
        SmartDashboard.putNumber("Current Speed of " + Name, getCurrentSpeed());
        SmartDashboard.putNumber("Current Encoder Rotation of " + Name, getSwerveEncoderAngleDegrees());
        SmartDashboard.putNumber("SparkMaxPower", DriveMotor.get());
        SmartDashboard.putNumber("Encoder Speed", altSwerveEncoder.getVelocity());
        SmartDashboard.putNumber("Target Speed of " + Name, getTargetSpeed());
        SmartDashboard.putNumber("Target Angle of " + Name, Math.toDegrees(getTargetEncoderLocation()));
    }
}

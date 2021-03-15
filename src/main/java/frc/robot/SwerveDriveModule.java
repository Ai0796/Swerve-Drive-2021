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

    /*Constants for use in the swerve drive
    */

     //Encoder Voltage
     public static final double MAX_ENCODER_VOLTAGE = 5;

     //WHEEL RADIUS
     public static final double WHEEL_RADIUS = 2;

     //Drive encoder
     public static final double ENCODER_RESOLUTION = 42;

     //Swerve Gear radius in inches
     public static final double SwerveGearRadius = 1.75;
     
     //Wheel size radius in inches
     public static final double WheelRadius = 2;

     //Max drive speed velocity
     public static final double MAX_DRIVE_RPM = 2500;

     //Wheelbase size
     public static final double wheelbaseWidth = 2; //Change when final robot finished
     public static final double wheelbaseHeight = 3; //Change when final robot finished

     public static final boolean LEFT_IS_INVERTED = false;
     public static final boolean RIGHT_IS_INVERTED = true;

     //Gear ratio between Swerve gear and encoder gear
     public static final double ENCODER_GEAR_RATIO = 1;

     //PID Using Ziegler-Nicols Tuning
     //https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method68858
     private static final double DRIVE_ULTIMATE_GAIN = 1;
     private static final double DRIVE_OSCILLATION_PERIOD = 0.05;
     public static final double DRIVE_P = 0.6 * DRIVE_ULTIMATE_GAIN;
     public static final double DRIVE_I = 1.2 * DRIVE_ULTIMATE_GAIN / DRIVE_OSCILLATION_PERIOD;
     public static final double DRIVE_D = 0.075 * DRIVE_ULTIMATE_GAIN * DRIVE_OSCILLATION_PERIOD;


     private static final double SWERVE_ULTIMATE_GAIN = 1;
     private static final double SWERVE_OSCILLATION_PERIOD = 0.05;
     public static final double SWERVE_P = 0.6 * SWERVE_ULTIMATE_GAIN;
     public static final double SWERVE_I = 1.2 * SWERVE_ULTIMATE_GAIN/SWERVE_OSCILLATION_PERIOD;
     public static final double SWERVE_D = 0.075 * SWERVE_ULTIMATE_GAIN * SWERVE_OSCILLATION_PERIOD;

     //Constants to use for Swerve Motion Profile
     //Measured in rotations per second of the motor
     public static final double MAX_SWERVE_ANGULAR_VELOCITY = 195.8333333;
     //Measured in rotations per second squared of the motor
     public static final double MAX_SWERVE_ANGULAR_ACCELERATION = 23500;
    /*Constants Finished
    */

    
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

    private final double gear_ratio = 33.75;

    private final PIDController drivePIDController = new PIDController(
        DRIVE_P, 
        DRIVE_I, 
        DRIVE_D);

    private final ProfiledPIDController swervePIDController =
        new ProfiledPIDController(
            SWERVE_P,
            SWERVE_I,
            SWERVE_D,
            new TrapezoidProfile.Constraints(
                MAX_SWERVE_ANGULAR_VELOCITY / gear_ratio, MAX_SWERVE_ANGULAR_ACCELERATION/gear_ratio));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
//   private final SimpleMotorFeedforward m_swerveFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
        swerveEncoder.setDistancePerRotation(ENCODER_GEAR_RATIO);
        driveEncoder = Drive_Motor.getEncoder();
        driveEncoder.setVelocityConversionFactor(1/MAX_DRIVE_RPM);
        Name = name;

        //Initialize Encoder Distances
        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);

        //Limit Swerve PID Controller to (-pi, pi)
        swervePIDController.enableContinuousInput(-Math.PI, Math.PI);

        //Set wheel Radius and Circumference
        wheelRadius = WHEEL_RADIUS;
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
        driveEncoder.setVelocityConversionFactor(1/MAX_DRIVE_RPM);
        Name = name;

        //Initialize Encoder Distances
        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);

        //Limit Swerve PID Controller to (-pi, pi)
        swervePIDController.enableContinuousInput(-Math.PI, Math.PI);

        //Set wheel Radius and Circumference
        wheelRadius = WHEEL_RADIUS;
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

            //Swerve Doesn't logically use a feed forward, implementation is kept in case
            double swerveFeedforward = 0;
            // double swerveFeedforward = m_swerveFeedforward.calculate(swerveVelocity);


            setDriveSpeed(driveOutput * 0.15);
            setEncoderSpeed(turnOutput + swerveFeedforward);
            // setDriveSpeed(0.75);
            // setEncoderSpeed(1);
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

    /*
    Calculates the target speed vector of the wheel using the equation
    xf = x + (r * width)
    yf = y + (r * height)
    v = sqrt(xf^2 + yf^2)
    returns a positive double
    */
    public double wheelVectorCalculator(double x, double y, double r){
        double xVector;
        double yVector;
        double speedVector;
        xVector = x + (r * xDistance);
        yVector = y + (r * yDistance);
        speedVector = Math.sqrt(Math.pow(xVector, 2) + Math.pow(yVector, 2));

        return speedVector;
    }

    /*
    Calculates the Target angle of the swerve using the equation:
    xf = x + (r * width)
    yf = y + (r * height)
    v = sqrt(xf^2 + yf^2)
    angle = math.atan(yf/xf)
    returns double between -pi and pi
    */
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

    /*
    Divides the targetWheelSpeed by a number
    Used for normalization
    */
    public void divideVelocity(double denominator){
        targetWheelSpeed /= denominator;
    }
    
    /*
    Sets the actual encoder speed, exists in case we want to change the way 
    encoder speed is set
    */
    public void setEncoderSpeed(double speed){
        EncoderMotor.set(speed);
    }

    /*
    Sets the actual drive speed, exists in case we want to change the way 
    drive speed is set
    */
    public void setDriveSpeed(double speed){
        DriveMotor.set(speed);
    }

    /*
    Returns degree angle of swerve by converting radians
    */
    public double getSwerveEncoderAngleDegrees(){
        return Math.toDegrees(getSwerveEncoderAngleRadians());
    }

    /*
    Returns encoder value, works on either encoder
    Equation 1: wrap(Amount of Rotations / Gear_ratio from motor to swerve * 2pi (to convert rotations to radians))
    Equation 2: Encoder/Max_Encoder_Voltage * 2pi
    Absolute encoder works like a potentiometer, returns value between 0-5V mapped between 0-360 degrees
    Returns a number between -pi and pi
    */
    public double getSwerveEncoderAngleRadians(){
        // SmartDashboard.putNumber("Encoder Value", ((swerveEncoder.getDistance()/MAX_ENCODER_VOLTAGE) - 0.5) * Math.PI);
        if(usingAlternativeEncoder){
            return EncoderGear.WrapRadians(altSwerveEncoder.getPosition() / gear_ratio * 2 * Math.PI);
        }
        return ((swerveEncoder.getDistance()/MAX_ENCODER_VOLTAGE)) * 2 * Math.PI;
    }


    /*
    Alternative faster way to get current stored encoder location
    Used for Smart Dashboard
    */
    public double getCurrentEncoderLocation(){
        return EncoderGear.getRadians();
    }

    public double getTargetEncoderLocation(){
        return targetEncoderLocation;
    }

    public double getCurrentRPM(){
        return driveEncoder.getVelocity();
    }

    /*
    RPM * Circumference
    */
    public double getCurrentSpeed(){
        return driveEncoder.getVelocity() * wheelCircumference;
    }

    public double getTargetSpeed(){
        return targetWheelSpeed;
    }

    public void UpdateSD(){
        SmartDashboard.putNumber("Current Speed of " + Name, getCurrentSpeed());
        SmartDashboard.putNumber("Current Encoder Rotation of " + Name, getSwerveEncoderAngleDegrees());
        SmartDashboard.putNumber("SparkMaxPower of" + Name, DriveMotor.get());
        // SmartDashboard.putNumber("Encoder Speed", altSwerveEncoder.getVelocity());
        SmartDashboard.putNumber("Target Speed of " + Name, getTargetSpeed());
        SmartDashboard.putNumber("Target Angle of " + Name, Math.toDegrees(getTargetEncoderLocation()));
    }
}

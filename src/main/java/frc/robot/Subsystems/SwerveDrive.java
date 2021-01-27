package frc.robot.Subsystems;

import java.lang.FdLibm.Pow;

import javax.print.CancelablePrintJob;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;

import frc.robot.CircleGeometry;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwerveDrive {
    

    private final CANSparkMax UL_DriveMotor, UR_DriveMotor, LL_DriveMotor, LR_DriveMotor;
    private final CANSparkMax UL_EncoderMotor, UR_EncoderMotor, LL_EncoderMotor, LR_EncoderMotor;
    private final CircleGeometry UL_Swerve, UR_Swerve, LL_Swerve, LR_Swerve;
    
    private final double DriveGearRatio;
    private final double EncoderGearRatio;
    private static SwerveDrive instance;

    private double kEpsilon = 1e-12;

    public SwerveDrive(){

        //Drive Motors
        UL_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.UL_DRIVE, MotorType.kBrushless);
        UR_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.UR_DRIVE, MotorType.kBrushless);
        LL_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.LL_DRIVE, MotorType.kBrushless);
        LR_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.LR_DRIVE, MotorType.kBrushless);

        //Encoder Motors
        UL_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.UL_ENCODER, MotorType.kBrushless);
        UR_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.UR_ENCODER, MotorType.kBrushless);
        LL_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.LL_ENCODER, MotorType.kBrushless);
        LR_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.LR_ENCODER, MotorType.kBrushless);

        //Encoder Positions
        UL_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.UL_Position);
        UR_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.UR_Position);
        LL_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.LL_Position);
        LR_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.LR_Position);


    }

    /**
     * creates a new instance of the drivetrain class if one has not been made
     * @return an instance of the drivetrain class
     */
    public static SwerveDrive getInstance() {
        if (instance == null) {
            return new SwerveDrive();
        }
        return instance;
    }

    private void setSpeed(CANSparkMax Motor, double speed){
        Motor.set(speed);
    }

    private void setSwerveAngle(CANSparkMax Motor, CircleGeometry currentCircle, double finalLocation){
        //Hacked in method, change to motion profile instead of just turning
        double distance = finalLocation - currentCircle.getRadians();
        double distanceTravelled = 0;
        double encoderLocation = //Insert encoder here
        if(distance >= 0){
            Motor.set(1);
        }else{
            Motor.set(-1);
        }
        while(distanceTravelled < distance){
            wait(2);
        }
        Motor.set(0);
        
    }

    /*Takes in an 
    x - horizontal
    y - vertical
    r - rotation
    and calculations swerve rotation and speeds
    */

    public void swerveDriveCalculation(double x, double y, double r){
        double ULSpeed;
        double URSpeed;
        double LLSpeed;
        double LRSpeed;

        double ULAngle;
        double URAngle;
        double LLAngle;
        double LRAngle;

        double maxSpeed;

        ULSpeed = wheelVectorCalculator(x, y, r, RobotMap.SwerveDrive.UL_XLOCATION, RobotMap.SwerveDrive.UL_YLOCATION);
        URSpeed = wheelVectorCalculator(x, y, r, RobotMap.SwerveDrive.UR_XLOCATION, RobotMap.SwerveDrive.UR_YLOCATION);
        LLSpeed = wheelVectorCalculator(x, y, r, RobotMap.SwerveDrive.LL_XLOCATION, RobotMap.SwerveDrive.LL_YLOCATION);
        LRSpeed = wheelVectorCalculator(x, y, r, RobotMap.SwerveDrive.LR_XLOCATION, RobotMap.SwerveDrive.LR_YLOCATION);

        ULAngle = wheelAngleCalculator(x, y, r, RobotMap.SwerveDrive.UL_XLOCATION, RobotMap.SwerveDrive.UL_YLOCATION);
        URAngle = wheelAngleCalculator(x, y, r, RobotMap.SwerveDrive.UR_XLOCATION, RobotMap.SwerveDrive.UR_YLOCATION);
        LLAngle = wheelAngleCalculator(x, y, r, RobotMap.SwerveDrive.LL_XLOCATION, RobotMap.SwerveDrive.LL_YLOCATION);
        LRAngle = wheelAngleCalculator(x, y, r, RobotMap.SwerveDrive.LR_XLOCATION, RobotMap.SwerveDrive.LR_YLOCATION);

        //Checks to see if rotating backwards and inverting the motor would be faster
        if(!UL_Swerve.checkFastestTurn(ULAngle)){
            ULSpeed *= -1;
            ULAngle = UL_Swerve.WrapRadians(ULAngle + Math.PI);
        }

        if(!UR_Swerve.checkFastestTurn(URAngle)){
            URSpeed *= -1;
            URAngle = UR_Swerve.WrapRadians(URAngle + Math.PI);
        }

        if(!LL_Swerve.checkFastestTurn(LLAngle)){
            LLSpeed *= -1;
            LLAngle = LL_Swerve.WrapRadians(LLAngle + Math.PI);
        }

        if(!LR_Swerve.checkFastestTurn(LRAngle)){
            LRSpeed *= -1;
            LRAngle = LR_Swerve.WrapRadians(LRAngle + Math.PI);
        }

        //Normalizes all values between -1 to 1 in order to feed into the motor
        maxSpeed = maxValue(ULSpeed, URSpeed, LLSpeed, LRSpeed);
        ULSpeed /= maxSpeed;
        URSpeed /= maxSpeed;
        LLSpeed /= maxSpeed;
        LRSpeed /= maxSpeed;

        setSwerveAngle(UL_EncoderMotor ,UL_Swerve, ULAngle);
        setSpeed(UL_DriveMotor, ULSpeed);
    }

    private double maxValue(double UL, double UR, double LL, double LR){
        double maxValue;
        maxValue = Math.max(Math.abs(UL), Math.abs(UR));
        maxValue = Math.max(maxValue, Math.abs(LL));
        maxValue = Math.max(maxValue, Math.abs(LR));
        return maxValue;
    }

    public double wheelVectorCalculator(double x, double y, double r, double xLocation, double yLocation){
        double xVector;
        double yVector;
        double speedVector;
        xVector = x + (r * xLocation);
        yVector = y + (r * yLocation);
        speedVector = Math.sqrt(Math.pow(xVector, 2) + Math.pow(yVector, 2));

        return speedVector;
    }


    /*
    Returns calculated wheel rotation in radians
    */
    public double wheelAngleCalculator(double x, double y, double r, double xLocation, double yLocation){
        double xVector;
        double yVector;
        double angle;
        xVector = x + (r * xLocation);
        yVector = y + (r * yLocation);
        if(yVector < kEpsilon){
            angle = 0;
        }else{
            angle = Math.atan(xVector/yVector);
        }
        return angle;
    }


}

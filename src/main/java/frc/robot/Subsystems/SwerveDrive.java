package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CircleGeometry;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SwerveDriveModule;

public class SwerveDrive {
    

    // private final CANSparkMax UL_DriveMotor, UR_DriveMotor, LL_DriveMotor, LR_DriveMotor;
    // private final CANSparkMax UL_EncoderMotor, UR_EncoderMotor, LL_EncoderMotor, LR_EncoderMotor;
    // private final CircleGeometry UL_Swerve, UR_Swerve, LL_Swerve, LR_Swerve;
    // private final SwerveDriveModule UL_SwerveModule, UR_SwerveModule, LL_SwerveModule, LR_SwerveModule;
    // private final AnalogInput UL_Input, UR_Input, LL_Input, LR_Input;
    // private final AnalogEncoder UL_Encoder, UR_Encoder, LL_Encoder, LR_Encoder;

    private final CANSparkMax UL_DriveMotor;
    private final CANSparkMax UL_EncoderMotor;
    private final CircleGeometry UL_Swerve;
    private final SwerveDriveModule UL_SwerveModule;
    private final AnalogInput UL_Input;
    private final AnalogEncoder UL_Encoder; 
    
    // private final double DriveGearRatio;
    // private final double EncoderGearRatio;
    private static SwerveDrive instance;

    double kEpsilon = 1e-13;

    public SwerveDrive(){

        //Drive Motors
        UL_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.UL_DRIVE_MOTOR, MotorType.kBrushless);
        // UR_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.UR_DRIVE_MOTOR, MotorType.kBrushless);
        // LL_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.LL_DRIVE_MOTOR, MotorType.kBrushless);
        // LR_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.LR_DRIVE_MOTOR, MotorType.kBrushless);

        //Encoder Motors
        UL_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.UL_ENCODER_MOTOR, MotorType.kBrushless);
        // UR_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.UR_ENCODER_MOTOR, MotorType.kBrushless);
        // LL_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.LL_ENCODER_MOTOR, MotorType.kBrushless);
        // LR_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.LR_ENCODER_MOTOR, MotorType.kBrushless);

        //Encoder Positions
        UL_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.UL_Position);
        // UR_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.UR_Position);
        // LL_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.LL_Position);
        // LR_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.LR_Position);

        //Encoders
        UL_Input = new AnalogInput(RobotMap.SwerveDrive.UL_ENCODER);
        UL_Encoder = new AnalogEncoder(UL_Input);

        // UR_Input = new AnalogInput(RobotMap.SwerveDrive.UR_ENCODER);
        // UR_Encoder = new AnalogEncoder(UR_Input);

        // LL_Input = new AnalogInput(RobotMap.SwerveDrive.LL_ENCODER);
        // LL_Encoder = new AnalogEncoder(LL_Input);

        // LR_Input = new AnalogInput(RobotMap.SwerveDrive.LR_ENCODER);
        // LR_Encoder = new AnalogEncoder(LR_Input);

        

        /*Format:
        {Name}_SwerveModule = new SwerveDriveModule({Name}_Swerve, {Name}_DriveMotor, {Name}_EncoderMotor,
        xPosition, yPosition,{Name}_Encoder, "{Full Name}");
        */

        UL_SwerveModule = new SwerveDriveModule(UL_Swerve, UL_DriveMotor, UL_EncoderMotor, 
        -RobotMap.SwerveDrive.wheelbaseWidth/2, RobotMap.SwerveDrive.wheelbaseHeight/2, UL_Encoder, "Upper Left");

        // UR_SwerveModule = new SwerveDriveModule(UR_Swerve, UR_DriveMotor, UR_EncoderMotor,
        // RobotMap.SwerveDrive.wheelbaseWidth/2, RobotMap.SwerveDrive.wheelbaseHeight/2,
        // UR_Encoder, "Upper Right");

        // LL_SwerveModule = new SwerveDriveModule(LL_Swerve, LL_DriveMotor, LL_EncoderMotor, 
        // -RobotMap.SwerveDrive.wheelbaseWidth/2, -RobotMap.SwerveDrive.wheelbaseHeight/2, 
        // LL_Encoder, "Upper Left");

        // LR_SwerveModule = new SwerveDriveModule(LR_Swerve, LR_DriveMotor, LR_EncoderMotor, 
        // RobotMap.SwerveDrive.wheelbaseWidth/2, -RobotMap.SwerveDrive.wheelbaseHeight/2, 
        // LR_Encoder, "Upper Left");
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


    /*Takes in an 
    x - horizontal
    y - vertical
    r - rotation
    and calculations swerve rotation and speeds
    */

    public void drive(){

        // double denominator; //To normalize all targets to between -1 and 1
        double x = OI.driver.getLX();
        double y = OI.driver.getLY();
        double r = OI.driver.getRX();
        CircleGeometry convertedXY;

        // SmartDashboard.putNumber("Encoder Value", ((UL_Encoder.getDistance()/RobotMap.SwerveDrive.MAX_ENCODER_VOLTAGE) - 0.5) * Math.PI);

        // convertedXY = convertInputs(x, y);
        // x = convertedXY.sin() * convertedXY.getRadius();
        // y = convertedXY.cos() * convertedXY.getRadius();


        UL_SwerveModule.setVelocity(x, y, r);
        // UR_SwerveModule.setVelocity(x, y, r);
        // LL_SwerveModule.setVelocity(x, y, r);
        // LR_SwerveModule.setVelocity(x, y, r);
        // // Normalizes values so all motors move to max proportional speed
        // denominator = maxValue(UL_SwerveModule, UR_SwerveModule, LL_SwerveModule, LR_SwerveModule);

        // UL_SwerveModule.divideVelocity(denominator);
        // UR_SwerveModule.divideVelocity(denominator);
        // LL_SwerveModule.divideVelocity(denominator);
        // LR_SwerveModule.divideVelocity(denominator);

        UL_SwerveModule.setDesiredVelocity();
        // UR_SwerveModule.goToVelocity();
        // LL_SwerveModule.goToVelocity();
        // LR_SwerveModule.goToVelocity();
    }

    public void test(){
        // double denominator; //To normalize all targets to between -1 and 1
        double y = OI.driver.getLY();
        double y2 = OI.driver.getRY();


        UL_SwerveModule.setDriveSpeed(y);
        UL_SwerveModule.setEncoderSpeed(y2);
    }

    public void testPID(double[] PID){
        double x = OI.driver.getLX();
        double y = OI.driver.getLY();
        double r = OI.driver.getRX();
        CircleGeometry convertedXY;

        // SmartDashboard.putNumber("Encoder Value", ((UL_Encoder.getDistance()/RobotMap.SwerveDrive.MAX_ENCODER_VOLTAGE) - 0.5) * Math.PI);

        // convertedXY = convertInputs(x, y);
        // x = convertedXY.sin() * convertedXY.getRadius();
        // y = convertedXY.cos() * convertedXY.getRadius();


        UL_SwerveModule.setVelocity(x, y, r);

        UL_SwerveModule.setDesiredVelocityTEST(PID[0], PID[1], PID[2]);
    }

    /*Uses Gyro to configure inputs in order for the robot 
    to always move in the same x and y directions
    */
    public CircleGeometry convertInputs(double x, double y){
        double angle;
        double radius;
        
        if (Math.abs(x) < kEpsilon){
            if(y > 0){
                angle = Math.PI/2;
            }else{
                angle = 3 * Math.PI/2;
            }
        }else{
            angle = Math.atan(y/x);
        }
        // angle += Math.toRadians(Robot.gyro.getAngle());
        radius = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return new CircleGeometry(radius, angle);
        
    }

    // private double maxValue(SwerveDriveModule UL, SwerveDriveModule UR, SwerveDriveModule LL, SwerveDriveModule LR){
    //     double maxValue;
    //     maxValue = Math.max(Math.abs(UL.getTargetSpeed()), Math.abs(UR.getTargetSpeed()));
    //     maxValue = Math.max(maxValue, Math.abs(LL.getTargetSpeed()));
    //     maxValue = Math.max(maxValue, Math.abs(LR.getTargetSpeed()));
    //     return maxValue;
    // }

    public void updateSD(){
        UL_SwerveModule.UpdateSD();
        // UR_SwerveModule.UpdateSD();
        // LL_SwerveModule.UpdateSD();
        // LR_SwerveModule.UpdateSD();
    }
}

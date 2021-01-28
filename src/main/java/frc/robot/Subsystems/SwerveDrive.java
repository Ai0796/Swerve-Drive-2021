package frc.robot.Subsystems;

import javax.print.CancelablePrintJob;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.CircleGeometry;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SwerveDriveModule;

public class SwerveDrive {
    

    private final CANSparkMax UL_DriveMotor, UR_DriveMotor, LL_DriveMotor, LR_DriveMotor;
    private final CANSparkMax UL_EncoderMotor, UR_EncoderMotor, LL_EncoderMotor, LR_EncoderMotor;
    private final CircleGeometry UL_Swerve, UR_Swerve, LL_Swerve, LR_Swerve;
    private final SwerveDriveModule UL_SwerveModule;
    private final AnalogInput UL_Input;
    private final AnalogEncoder UL_Encoder;
    
    // private final double DriveGearRatio;
    // private final double EncoderGearRatio;
    private static SwerveDrive instance;

    public SwerveDrive(){

        //Drive Motors
        UL_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.UL_DRIVE_MOTOR, MotorType.kBrushless);
        UR_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.UR_DRIVE_MOTOR, MotorType.kBrushless);
        LL_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.LL_DRIVE_MOTOR, MotorType.kBrushless);
        LR_DriveMotor = new CANSparkMax(RobotMap.SwerveDrive.LR_DRIVE_MOTOR, MotorType.kBrushless);

        //Encoder Motors
        UL_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.UL_ENCODER_MOTOR, MotorType.kBrushless);
        UR_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.UR_ENCODER_MOTOR, MotorType.kBrushless);
        LL_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.LL_ENCODER_MOTOR, MotorType.kBrushless);
        LR_EncoderMotor = new CANSparkMax(RobotMap.SwerveDrive.LR_ENCODER_MOTOR, MotorType.kBrushless);

        //Encoder Positions
        UL_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.UL_Position);
        UR_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.UR_Position);
        LL_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.LL_Position);
        LR_Swerve = new CircleGeometry(RobotMap.SwerveDrive.SwerveGearRadius, RobotMap.SwerveDrive.LR_Position);

        //Encoders
        UL_Input = new AnalogInput(RobotMap.SwerveDrive.UL_ENCODER);
        UL_Encoder = new AnalogEncoder(UL_Input);
        

        UL_SwerveModule = new SwerveDriveModule(UL_Swerve, UL_DriveMotor, UL_EncoderMotor, -RobotMap.SwerveDrive.wheelbaseWidth/2, RobotMap.SwerveDrive.wheelbaseHeight/2, UL_Encoder, "Upper Left");
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

        double x = OI.driver.getLX();
        double y = OI.driver.getLY();
        double r = OI.driver.getRX();



        UL_SwerveModule.setVelocity(x, y, r);
        // Normalizes values so all motors move to max proportional speed
        // maxValue(SwerveDriveModule UL, SwerveDriveModule UR, SwerveDriveModule LL, SwerveDriveModule LR);
        UL_SwerveModule.goToVelocity();
    }

    private double maxValue(SwerveDriveModule UL, SwerveDriveModule UR, SwerveDriveModule LL, SwerveDriveModule LR){
        double maxValue;
        maxValue = Math.max(Math.abs(UL.getTargetSpeed()), Math.abs(UR.getTargetSpeed()));
        maxValue = Math.max(maxValue, Math.abs(LL.getTargetSpeed()));
        maxValue = Math.max(maxValue, Math.abs(LR.getTargetSpeed()));
        return maxValue;
    }

    public void updateSD(){
        UL_SwerveModule.UpdateSD();
    }
}

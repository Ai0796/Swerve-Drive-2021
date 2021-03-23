package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.SwerveDrive;
import frc.robot.helpers.ControllerWrapper;

/**
 * The class in which we map our driver/operator input to specific tasks on the
 * robot Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * 
 * @author hrl
 */
public class OI {

    public static ControllerWrapper driver = new ControllerWrapper(RobotMap.Controllers.DRIVER_PORT, true);

    //Used for adjusting PID mid turn
    public static double PIDVar[] = {0, 0, 0};
    public static int PIDSelector = 0; //Default to adjusting P

    public static void init() {
        // initSD();
    }

    public static void update() {

        // GENERAL CONTROLS/CONTROL METHODS
        SmartDashboard.putNumber("Left Stick X Axis", driver.getLX());
        SmartDashboard.putNumber("Left Stick Y Axis", driver.getLY());
        SmartDashboard.putNumber("Right Stick X Axis", driver.getRX());
        SmartDashboard.putNumber("PID Change Selected", PIDSelector);
        SmartDashboard.putNumber("P", PIDVar[0]);
        SmartDashboard.putNumber("I", PIDVar[1]);
        SmartDashboard.putNumber("D", PIDVar[2]);



        // Robot.swerveDrive.drive();
        Robot.swerveDrive.testPID(PIDVar);

        if(driver.bA.get()){
            PIDSelector = 0;
        }
        else if(driver.bB.get()){
            PIDSelector = 1;
        }
        else if(driver.bY.get()){
            PIDSelector = 2;
        }
        if(driver.bRT.get()){
            PIDVar[PIDSelector] += 0.01;
        }
        else if(driver.bLT.get()){
            PIDVar[PIDSelector] -= 0.01;
        }

        updateSD();
        // updateSDTesting();
    }

    public static void testupdate() {
        // GENERAL CONTROLS/CONTROL METHODS
        SmartDashboard.putNumber("Left Stick X Axis", driver.getLX());
        SmartDashboard.putNumber("Left Stick Y Axis", driver.getLY());
        SmartDashboard.putNumber("Right Stick X Axis", driver.getRX());

        Robot.swerveDrive.test();

        // updateSD();
        updateSDTesting();
    }

    // public static void initSD() {
    //     SmartDashboard.putNumber("Upper Left Wheel Speed", Robot.swerveDrive.)
    // }

    // /**
    //  * Used for updating the SmartDashboard during the match.
    //  */
    public static void updateSD() {

        Robot.swerveDrive.updateSD();
        // SmartDashboard.putBoolean("Bot Left Sensor", Robot.hopper.getBotLeftSensor());
        // SmartDashboard.putBoolean("Bot Right Sensor", Robot.hopper.getBotRightSensor());
        // SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        // SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());

        // SmartDashboard.putBoolean("Shooter running?", Robot.shooter.getRPM() > 0);
        // SmartDashboard.putBoolean("At target", Robot.shooter.atTarget());

        // SmartDashboard.putBoolean("Alt Mode", operator.isAltMode());

        // SmartDashboard.putNumber("Shooter RPM", Robot.shooter.getRPM());

        SmartDashboard.updateValues();
    }

    /**
     * Used for updating the SmartDashboard during testing/troubleshooting.
     */
    public static void updateSDTesting() {
        Robot.swerveDrive.updateSD();
        // SmartDashboard.putNumber("Hopper count", Robot.hopper.getBallCount());

        // SmartDashboard.putBoolean("Turret left", Robot.turret.getLeftLimit());
        // SmartDashboard.putBoolean("Turret right", Robot.turret.getRightLimit());

        // SmartDashboard.putNumber("Turret Speed", Robot.turret.getSpeed());
        // SmartDashboard.putNumber("Turret position", Robot.turret.getCompEncPosition());
        // SmartDashboard.putNumber("Limelight X Pos", Robot.camera.getPosition());
        // SmartDashboard.putNumber("Calculated RPM", Robot.camera.getCalculatedRPM());

        // SmartDashboard.putNumber("CL_Winch", Robot.climber.getWinchPosition());
        // SmartDashboard.putNumber("CL_Arm", Robot.climber.getArmPosition());
        // SmartDashboard.putBoolean("CL_Sensor", Robot.climber.getArmDeploy());

        // SmartDashboard.putBoolean("Operator Alt Mode", operator.isAltMode());
        Robot.swerveDrive.updateSD();
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Robot;

/**
 * This class rotates the robot to the specified angle (in degrees) using a Gyro sensor.
 * It uses a PID controller to accomplish this behavior.
 * 
 * @author jw, gjs
 */
public class TurnToAngleGyro {
    private static final double kP = 0.0039, kI = 0, kD = 0; // TODO: untested
    private final double tolerance = 1; // TODO: untested
    private PIDController pid;

    private double initialAngle;

    /**
     * Creates a new TurnToAngleGyro object, to be run with TurnToAngleGyro.run().
     * @param angle the angle to turn to, from -180 to +180
     */
    public TurnToAngleGyro() {
        // initialize sources
        pid = new PIDController(kP, kI, kD);

        this.initialAngle = Robot.gyro.getAngle();
    }

    /**
     * Resets the initial angle to wherever the robot currently is.
     */
    public void reset() {
        this.initialAngle = Robot.gyro.getAngle();
    }

    /**
     * Turns the robot to the specified angle (in degrees).
     */
    public void run(int angle) {
        double currentAngle = Robot.gyro.getAngle() - initialAngle;

        if (angle > 0 && angle <= 180) {
            if (currentAngle < angle - tolerance || currentAngle > angle + tolerance) {
                double speed = pid.calculate(currentAngle, angle);
                Robot.drivetrain.setSpeed(speed, -speed);
            }
        } else if (angle < 0 && angle >= -180) { // turn backwards
            if (currentAngle < angle - tolerance || currentAngle > angle + tolerance) {
                double speed = pid.calculate(currentAngle, angle);
                Robot.drivetrain.setSpeed(speed, -speed);
            }
        } else {
            System.out.println("TTAG: out of range");
        }
    }
}

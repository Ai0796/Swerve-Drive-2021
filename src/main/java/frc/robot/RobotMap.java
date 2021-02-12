package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * This is the RobotMap class, which stores constants for use throughout the
 * project. These constants include motor controller constants, controller
 * mappings, etc.
 * @author common
 */

public class RobotMap {

    public static class Common {
        //Put stuff in here that can be used for all classes, like the refresh rate
        public static final double UPDATE_PERIOD = 1/60;
    }

    public static class Camera {
        public static final double PIPELINE = 0.0;
        public static final double OFFSET_SCALE = -0.05285; // TODO: tune this with more datapoints
    }

    public static class SwerveDrive {
        public static final int UL_DRIVE_MOTOR = 12;
        public static final int UR_DRIVE_MOTOR = 13;
        public static final int LL_DRIVE_MOTOR = 14;
        public static final int LR_DRIVE_MOTOR = 15;

        public static final int UL_ENCODER_MOTOR = 16;
        public static final int UR_ENCODER_MOTOR = 17;
        public static final int LL_ENCODER_MOTOR = 18;
        public static final int LR_ENCODER_MOTOR = 19;

        //Current location in radians from -pi to pi
        public static final double UL_Position = 0;
        public static final double UR_Position = 0;
        public static final double LL_Position = 0;
        public static final double LR_Position = 0;

        //We're using these: https://www.andymark.com/products/ma3-absolute-encoder-with-cable Encoders
        public static final int UL_ENCODER = 0;
        public static final int UR_ENCODER = 1;
        public static final int LL_ENCODER = 2;
        public static final int LR_ENCODER = 3;

        //Encoder Voltage
        public static final double MAX_ENCODER_VOLTAGE = 5;

        //WHEEL RADIUS
        public static final double WHEEL_RADIUS = 2;

        //Drive encoder
        public static final double ENCODER_RESOLUTION = 4096;

        //Swerve Gear radius in inches
        public static final double SwerveGearRadius = 1.75;
        
        //Wheel size radius in inches
        public static final double WheelRadius = 2;

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
        private static final double DRIVE_OSCILLATION_PERIOD = 0.25;
        public static final double DRIVE_P = 0.6 * DRIVE_ULTIMATE_GAIN;
        public static final double DRIVE_I = 1.2 * DRIVE_ULTIMATE_GAIN / DRIVE_OSCILLATION_PERIOD;
        public static final double DRIVE_D = 0.075 * DRIVE_ULTIMATE_GAIN * DRIVE_OSCILLATION_PERIOD;


        private static final double SWERVE_ULTIMATE_GAIN = 1;
        private static final double SWERVE_OSCILLATION_PERIOD = 0.25;
        public static final double SWERVE_P = 0.6 * SWERVE_ULTIMATE_GAIN;
        public static final double SWERVE_I = 1.2 * SWERVE_ULTIMATE_GAIN/SWERVE_OSCILLATION_PERIOD;
        public static final double SWERVE_D = 0.075 * SWERVE_ULTIMATE_GAIN * SWERVE_OSCILLATION_PERIOD;

        //Constants to use for Swerve Motion Profile
        public static final double MAX_SWERVE_ANGULAR_VELOCITY = 5;
        public static final double MAX_SWERVE_ANGULAR_ACCELERATION = 5;
    }

    public static class Hopper {
        public static class Speeds {
            public static class CompBot {
                public static class Update {
                    public static final double BELT_SPEED = 0.5;
                    public static final double FEEDER_SPEED = 0.8;
                    public static final double BLUE_SPEED = 0.6;
                }
            }            
            public static class PracticeBot {
                public static class Update {
                    public static final double BELT_SPEED = 0.5;
                    public static final double FEEDER_SPEED = 0.8;
                    public static final double BLUE_SPEED = 0.6;
                }
            }
            public static class Forward {
                public static final double BELT_SPEED = 0.6;
                public static final double FEEDER_SPEED = 1;
                public static final double BLUE_SPEED = 0.7;
            }
        }

        public static class Sensors {
            public static final int BOT_SENSOR_PORT_LEFT = 5;
            public static final int BOT_SENSOR_PORT_RIGHT = 4;
            public static final int MID_LEFT = 3; // TODO: untested
            public static final int TOP_LEFT = 2; // TODO: untested

            public static final boolean TOP_LAST_STATE_VALUE = false; // TODO: untested
            public static final boolean MID_LAST_STATE_VALUE = false; // TODO: untested
            public static final boolean BOT_LAST_STATE_VALUE = true; // TODO: untested
        }

        public static class Motor {
            public static final int HOPPER_FLOOR = 10; // TODO: untested
            public static final int BLUE_WHEELS = 11; // TODO: untested
            public static final int FEEDER = 3; // TODO: untested
            public static final boolean HOPPER_FLOOR_IS_INVERTED = true; // TODO: untested
            public static final boolean FEEDER_IS_INVERTED = true; // TODO: untested
        }
    }

    public static class Intake {
        public static final int MOTOR = 1; // TODO: untested
        public static final boolean MOTOR_IS_INVERTED = true; // TODO: untested
        public static final int ACTUATOR1 = 3; // TODO: untested
        public static final int ACTUATOR2 = 7; // TODO: untested
    }

    public static class LED {
        public static final int PORT = 4; // TODO: untested
        public static final int LENGTH = 27; // TODO: untested
    }

    public static class Shooter {
        public static final int PRIMARY = 7; // TODO: untested
        public static final int SECONDARY = 8; // TODO: untested
        public static final boolean PRIMARY_IS_INVERTED = true; // TODO: untested
        public static final boolean SECONDARY_IS_OPPOSITE = true; // TODO: untested
        public static final double TRENCH_FRONT_SPD = 4300;// 4824.6;
    }

    public static class Turret {
        public static final int MOTOR = 2;

        public static class CompBot {
            public static final boolean MOTOR_IS_INVERTED = false;
            public static final int ENCODER_1 = 0;
            public static final int ENCODER_2 = 1;

            public static class Limits {
                public static final int LEFT_PORT = 6; // TODO: untested
                public static final int RIGHT_PORT = 7; // TODO: untested

                public static final boolean LIMIT_TRIGGERED = false;
            }

            public static class PID {
                public static final double kPv = 0.025;
                public static final double kIv = 0.007;
                public static final double kDv = 0.0008;

                public static final double VISION_OFFSET_SCALE = 0.093143;

                public static final double kP = 0; // TODO: untested
                public static final double kI = 0; // TODO: untested
                public static final double kD = 0; // TODO: untested
                
                public static final double GYRO_TO_ENCODER = 1; // TODO: untested
            }
                        
            public static class Setpoints {
                public static final double SAFE_POSITION = 0; // TODO: untested
                public static final double DEADZONE_LEFT = 0; // TODO: untested
                public static final double DEADZONE_RIGHT = 23000;
            }
        }

        public static class PracticeBot {
            public static final boolean MOTOR_IS_INVERTED = true; // TODO: untested

            public static class Limits {
                public static final int LEFT_PORT = 0; // TODO: untested
                public static final int RIGHT_PORT = 1; // TODO: untested

                public static final boolean LIMIT_TRIGGERED = true;
            }

            public static class PID {
                public static final double kPv = 0.015; // TODO: untested
                public static final double kIv = 0; // TODO: untested
                public static final double kDv = 0.001; // TODO: untested

                public static final double kP = 0; // TODO: untested
                public static final double kI = 0; // TODO: untested
                public static final double kD = 0; // TODO: untested

                public static final double GYRO_TO_ENCODER = 1; // TODO: untested
            }
            
            public static class Setpoints {
                public static final double SAFE_POSITION = 0; // TODO: untested
                public static final double DEADZONE_LEFT = 0; // TODO: untested
                public static final double DEADZONE_RIGHT = 23000; // TODO: untested
            }
        }
    }

    public static class WheelOfFortune {
        public static final int MOTOR = 4; // TODO: untested
        public static final boolean ACTUATOR_ENABLED_STATE = false; // TODO: untested

        public static final int ACTUATOR1 = 0; // TODO: untested
        public static final int ACTUATOR2 = 4; // TODO: untested

        /* NOTE: pneumatic solenoids for this subsystem have been moved to the climber as of the
        Bloomington competition. This may change depending on design decisions in the future */

        public static final I2C.Port COLOR_SENSOR = I2C.Port.kMXP; // TODO: untested
        public static final double[] RED_VALUES = {0.487, 0.360, 0.152};
        public static final double[] GREEN_VALUES = {0.197, 0.561, 0.240};
        public static final double[] BLUE_VALUES = {0.143, 0.427, 0.429};
        public static final double[] YELLOW_VALUES = {0.361, 0.524, 0.113};
        public static final double[] BLANK = {0, 0, 0};

    }

    public static class Climber {
        public static final int PRIMARY_WINCH = 5; // TODO: untested
        public static final int SECONDARY_WINCH = 6; // TODO: untested
        public static final boolean PRIMARY_WINCH_IS_INVERTED = true;

        public static final int ARM = 9; // TODO: untested
        public static final boolean ARM_IS_INVERTED = true;

        public static final int ACTUATOR1 = 1; // TODO: untested
        public static final int ACTUATOR2 = 6; // TODO: untested
        public static final int HOOK_ACTUATOR = 5;
        public static final int HOOK_ACTUATOR_UNUSED = 2;

        public static final double WINCH_SETPOINT = 0; // TODO: untested
        
        public static final int ARM_DEPLOY_SENSOR = 9;
        public static final boolean ARM_LAST_STATE_VALUE = true; // TODO: untested

        public static final double WINCH_HOLD_POWER = 0.5;
        public static final double ARM_POWER = 0.1;
    }

    public static class Sensors {
        public static final Port GYRO = SPI.Port.kMXP;
    }

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RB = 6;
        public static final int LB = 5;
        public static final int RSTK = 10;
        public static final int LSTK = 9;
        public static final int START = 8;
        public static final int MENU = 7;

        public static final int LX = 0;
        public static final int LY = 1; // Arcade drive
        public static final int RX = 2; // Arcade drive

        public static final int RY = 5;
        public static final int LT = 2;
        public static final int RT = 3;

        public static final int POV = 0; // untested
    }
}

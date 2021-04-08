package frc.robot.autonomous.paths;
import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.AutonPath;

public class Test{
    private static double total_distance = 0;
    private testState currentState = testState.ToPosition;
    private enum testState{
        ToPosition{
            public testState getNextState(){
                if(Robot.drivetrain.getRightPosition() == AutonMap.Test.DISTANCE){
                    return this;
                }
                return RightTurn;
            }
            @Override
            public String toString(){
                return "ToPosition";
            }
            public void run(){
                Robot.drivetrain.setSpeed(.5, .5);
            }
        },
        RightTurn{
            public testState getNextState(){
                // if(Robot.drivetrain.)
            }
        },
    }
    
}

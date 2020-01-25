package frc.robot;

import frc.parent.*;

//Extends the Mechanisms class
//Nothing to special
//Note: it only uses one talon, so using tTwo in anyway is cause an error
public class Climber{

    private static CCTalon tal = new CCTalon(RobotMap.CLIMBER, false);

    public static void set(double speed){
        tal.set(speed);
    }
    
}
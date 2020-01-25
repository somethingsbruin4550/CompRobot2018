package frc.robot;

import frc.parent.*;

//Extends the Mechanisms class
//It does use two talons so using tTwo won't cause an error(probably)
public class Intake{

    private static CCTalon tal1 = new CCTalon(RobotMap.INTAKE_A, true);
    private static CCTalon tal2 = new CCTalon(RobotMap.INTAKE_B, true);

    public static void set(double speed){
        tal1.set(speed);
        tal2.set(speed);
    }

}

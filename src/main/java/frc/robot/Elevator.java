package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.parent.*;
import frc.robot.CCSparkMax;

//Extends the Mechanisms class
//Nothing to special
//Note: it only uses one talon, so using tTwo in anyway is cause an error
public class Elevator{

    public CCSparkMax one;

    public Elevator(){
        one = new CCSparkMax(RobotMap.ELEVATOR, MotorType.kBrushless, IdleMode.kBrake, false);
    }

    public void set(double speed){
        one.set(speed);
    }

    public void setConst(double p, double i, double d, double Ff){
      one.setPID(p, i, d, Ff);
    }

    public void setPos(double pos){
        one.setPosition(pos);
    }




    
}
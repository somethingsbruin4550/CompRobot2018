package robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Elevator {
	TalonSRX talon1;
	Elevator _instance;
	DigitalInput upperLim;
	
	public Elevator(){
		talon1 = new TalonSRX(RobotMap.ELEVATOR);
		talon1.setNeutralMode(NeutralMode.Brake);
		upperLim = new DigitalInput(RobotMap.ELEVATOR_LIMIT);
	}
	
	public void setElevator(double speed){
		talon1.set(ControlMode.PercentOutput, speed);
	}
	
	public boolean getLimit() {
		return upperLim.get();
	}
	
	public void moveTime(double speed, double time) {
		setElevator(speed);
		Timer.delay(time);
		setElevator(0);
	}

	
}

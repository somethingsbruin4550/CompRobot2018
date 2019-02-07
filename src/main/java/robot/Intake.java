package robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class Intake {

	private TalonSRX _talon1;
	private TalonSRX _talon2;
//	DigitalInput _sensor;
//	DigitalOutput _led;
	 


	public Intake(){
		_talon1 = new TalonSRX(RobotMap.INTAKE_A);
		_talon2 = new TalonSRX(RobotMap.INTAKE_B);
		_talon1.setInverted(false);
		_talon2.setInverted(false);
//		_sensor = new DigitalInput(RobotMap.BREAK_SENSOR);
//		_led = new DigitalOutput(RobotMap.LED_PORT);
	}

	public void setIntake(double speed1, double speed2){
			_talon1.set(ControlMode.PercentOutput, speed1);
			_talon2.set(ControlMode.PercentOutput, speed2);
		
//			if(!breakCheck()) {
//				_led.pulse(50);
//			}
	}
	
	public void runIntake(double speed1, double speed2, double time) {
		setIntake(speed1, speed2);
		Timer.delay(time);
		setIntake( 0.0, 0.0 );
	}
//
//	public boolean breakCheck() {
//		return _sensor.get();
//	}



}

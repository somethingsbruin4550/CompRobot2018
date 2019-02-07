package robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class Climber {
	TalonSRX talon1;
	Encoder _encoder;
	
	public Climber(){
		talon1 = new TalonSRX(RobotMap.CLIMBER);
		talon1.setInverted(true);
		talon1.setNeutralMode(NeutralMode.Brake);
		_encoder = new Encoder(RobotMap.ENCODER_A_CLIMBER, RobotMap.ENCODER_B_CLIMBER);
	}
	
	
	public void setClimber(double speed){
		talon1.set(ControlMode.PercentOutput, speed);
	}
	
	public double getDistance() {
		return _encoder.getDistance();
	}
	
	public void reset() {
		_encoder.reset();
	}

	public void raiseClimber(double height, double dt, boolean debug) {
		//HEIGHT IS IN INCHES
		double feed_forward = 0; // forward input to reduce the steady state error
		double max = 0.5; // used to clamp the max speed, to slow down the robot
		double previous_error = 0; // used to calculate the derivative value
		double integral = 0; // used to carry the sum of the error
		double derivative = 0; // used to calculate the change between our goal and position
		double Kp = 0.005; // proportional constant
		double Ki = 0.0025; // integral constant
		double Kd = 0;//0.002; // derivative constant
		double goal = height*115.116;
		// this is what we want the robot to do: go forward,
		// turn, elevate, etc to a new position)
		double position = this.getDistance(); // current position in inches/feed, degrees, etc.)
		double error = 0; // our goal is to make the error from the current position zero)
		double error_check = goal/100;
		//System.out.println("Starting");
		
		while(true) {
			//Reset the Position
			position = this.getDistance();
			//Calculates the error based on how far the robot is from the dist
			error = goal - position;
			//Calculates the Integral based on the error, delay, and the previous integral 
			integral = integral + error * dt; 
			//Calculates the derivative based on the error and the delay 
			derivative = (error - previous_error) / dt;
			//MATH 
			//System.out.println("Output calcs1: " +  Kp * error + Ki * integral + Kd * derivative + feed_forward);
			double output = Kp * error + Ki * integral + Kd * derivative + feed_forward;
			//System.out.println("Output calcs2: " +  Kp * error + Ki * integral + Kd * derivative + feed_forward);
			//Passes on the error to the previous error
			previous_error = error;
			
			//NORMALIZE: If the spd is bigger than we want, set it to the max, if its less than the -max makes it the negitive max
			if(output > max)
				output = max;
			else if(output < -max)
				output = -max; 
			
			//After the spd has been fixed, set the speed to the output
			this.setClimber(output);
			
			//If it's close enough, just break and end the loop 
			if(error <= error_check) {
				//System.out.println("break");
				//break;
			}
			
			//Delay(Uses dt)
			Timer.delay(dt);
			if(debug) {
				
				System.out.println("Position: " + position);
				System.out.println("Error: " + error);
				System.out.println("Output: " + output);
				System.out.println("Integral: " + integral);
				
			}
		}
	}
	
	
	public double raiseConcurrently(double error, double previousError, double dt, double max, double previousIntegral, boolean debugOn) {
		double Kp = 0.00475; // proportional constant
		double Ki = 0; // integral constant
		double Kd = 0;//0.002; // derivative constant
		double integral = previousIntegral + error * dt; 
		//Calculates the derivative based on the error and the delay 
		double derivative = (error - previousError) / dt;
		//MATH 
		
		double output = Kp * error + Ki * integral + Kd * derivative;
		//Passes on the error to the previous error
		
	
		//NORMALIZE: If the spd is bigger than we want, set it to the max, if its less than the -max makes it the negative max
		if(output > max)
			output = max;
		else if(output < -max)
			output = -max; 
		
		//After the spd has been fixed, set the speed to the output
		this.setClimber(output);
		
		if(debugOn) {
			//System.out.println("Climber:");
			/*
			System.out.println("Integral: " + integral);
			System.out.println("Derivative: " + derivative);
			System.out.println("Output: " + output);
			*/
			//System.out.println(this.getDistance());
		}
		
		return error;
}

	
	
	
}

package frc.robot;

import frc.robot.RobotMap;
import frc.limelight.LimeCam;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Chassis {
	public Encoder _leftEncoder;
	public Encoder _rightEncoder;
	
	//	CCTalon _frontLeft;
	//	CCTalon _frontRight;
	//	CCTalon _rearLeft;
	//	CCTalon _rearRight;

	TalonSRX _frontLeft;
	TalonSRX _frontRight;
	TalonSRX _rearLeft;
	TalonSRX _rearRight;

	LimeCam limelight;
	AHRS _gyro;
	AnalogOutput _ultra;

	double offset = 0.05;
	private static Chassis _instance;
	//Creates the Chassis
	private Chassis() {
		limelight = new LimeCam();

		_frontLeft = new TalonSRX(RobotMap.LEFT_FORWARD_PORT);
		_frontRight = new TalonSRX(RobotMap.RIGHT_FORWARD_PORT);
		_rearLeft = new TalonSRX(RobotMap.LEFT_REAR_PORT);
		_rearRight = new TalonSRX(RobotMap.RIGHT_REAR_PORT);

		_frontLeft.setInverted(RobotMap.LEFT_FORWARD_REVERSE);
		_frontRight.setInverted(RobotMap.RIGHT_FORWARD_REVERSE);
		_rearLeft.setInverted(RobotMap.LEFT_REAR_REVERSE);
		_rearRight.setInverted(RobotMap.RIGHT_REAR_REVERSE);


		_frontLeft.setNeutralMode(NeutralMode.Brake);
		_frontRight.setNeutralMode(NeutralMode.Brake);
		_rearLeft.setNeutralMode(NeutralMode.Brake);
		_rearRight.setNeutralMode(NeutralMode.Brake);

		_leftEncoder = new Encoder(RobotMap.ENCODER_A_LEFT,
				RobotMap.ENCODER_B_LEFT);
		_rightEncoder = new Encoder(RobotMap.ENCODER_A_RIGHT, RobotMap.ENCODER_B_RIGHT);
		//		_ultra  = new AnalogOutput(RobotMap.ULTRA_PORT);

		try {
			_gyro = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException e) {
			DriverStation.reportError(
					"Error instantiating navX-MXP:  " + e.getMessage(), true);
		}
	}
	//Drive Method(Drives Robot)
	
	//Used to turn the Chassis at a speed for a time  
	public void turnTime(double spd, double time) {
		//Creates timer object, and starts the timer at 0 seconds 
		Timer clock = new Timer();
		clock.start();
		//Checks if the timer has not passed the inputed time, if so: turn the robot at the time 
		while(!clock.hasPeriodPassed(time)) {
			this.tankDrive(spd, 0.0);
			getAngle();
		} 
			this.tankDrive(0.0, 0.0);
			clock.stop();
			clock.reset();
	}
	public void drive(double xAxis, double yAxis) {
		// System.out.println("speedCheck: " + speedCheck(xAxis));
		xAxis += speedCheck(xAxis);
		// System.out.println("speedCheck(Y): " + speedCheck(yAxis));
		yAxis += speedCheck(yAxis);
		_frontLeft.set(ControlMode.PercentOutput, OI.normalize((yAxis + xAxis), -1.0, 0, 1.0));
		_frontRight.set(ControlMode.PercentOutput, OI.normalize((yAxis - xAxis), -1.0, 0, 1.0));
		_rearLeft.set(ControlMode.PercentOutput, OI.normalize((yAxis + xAxis), -1.0, 0, 1.0));
		_rearRight.set(ControlMode.PercentOutput, OI.normalize((yAxis - xAxis), -1.0, 0, 1.0));
		// System.out.println("xAxis: "+xAxis);
		// System.out.println("yAxis: "+yAxis);
	}
	//Tank Drive Method(Drives Robot, but better?)
	public void tankDrive(double xAxis, double yAxis) {
		_frontLeft.set(ControlMode.PercentOutput, OI.normalize((yAxis + xAxis), -1.0, 0, 1.0));
		_frontRight.set(ControlMode.PercentOutput, OI.normalize((yAxis - xAxis), -1.0, 0, 1.0));
		_rearLeft.set(ControlMode.PercentOutput, OI.normalize((yAxis + xAxis), -1.0, 0, 1.0));
		_rearRight.set(ControlMode.PercentOutput, OI.normalize((yAxis - xAxis), -1.0, 0, 1.0));

	}
	

	public double speedCheck(double Spd) {
		if (Spd > 1.0) {
			return -((Spd - 1.0) * 0.5);
		} else if (Spd < -1.0) {
			return -((Spd + 1.0) * 0.5);
		}
		return 0.0;
	}
	public void driveDistNonTrapezoidal(double distance, double speed) {
		reset();
		Timer t = new Timer();
		t.start();
		while ( Math.abs ( ( getLeftEncoder() + getRightEncoder() )/2) <= distance && t.get() < 3) {
			tankDrive(-0.005, speed);
//			System.out.println("Encoder: " + (( _leftEncoder.getDistance() + _rightEncoder.getDistance())/2));
		}
		tankDrive(0, 0.0);
		
		System.out.println("Encoder: " + (( _leftEncoder.getDistance() + _rightEncoder.getDistance())/2));
	}
	
	//Drives to a Distance 
	//Uses MATH to make us move a certain distance
	//@param Angle we want, P constant. accepError sets the acceptable error
//	public void driveDist(double goal, double p, double accepError) {
//		double goalEncoder = goal * 119.302; //Converts ft. to encoder values
//		double accepErrorEncoder = accepError * 119.302;
//		Timer clock = new Timer();
//		clock.start();
//		double dist;
//		double error;
//		double input;
//		while(true) {
//			dist = this.getRightEncoder();
//			error = goalEncoder - dist;
//			input = error * p;
////			System.out.print("Goal: " + goalEncoder + " ");
////			System.out.print("Distance: " + dist + " ");
////			System.out.print("Error: " + error + " ");
////			System.out.print("P: " + p + " ");
////			System.out.println("Input: " + input + " ");
////			System.out.println(this.getLeftEncoder());
////			System.out.println(this.getRightEncoder());
//			driveSpd(input);
//			Timer.delay(0.01);
//				
//			if(error <= accepErrorEncoder) {
//				clock.start();
//				if(clock.hasPeriodPassed(.5)) {
//					break;
//				}
//			}else if(error > accepErrorEncoder) {
//				clock.stop();
//			}
//		}
//	}
//	
	
	public void shuDrive(double dist, double percentError, double p, double max, boolean debug) {
		double goal = dist*119.301; // Converts the dist(feet) to encoder ticks
		double accepError = goal/percentError; // Uses percentError to determine the accepError
		double pos = (this.getRightEncoder() + this.getLeftEncoder())/2;
		double offsetL =  (pos - this.getLeftEncoder())/990.13; 
		double offsetR = (pos - this.getRightEncoder())/990.13;
		double error = goal - pos; 
		double input = 0; 
		while(true) {
			pos = (this.getRightEncoder() + this.getLeftEncoder())/2;
			error = goal - pos; 
			input = error*p; 
			if(input >= max) 
				input = max; 
				
			this.driveSpd(input+offsetL, input+offsetR);
			
			if(debug) {
				System.out.println("Pos: " + pos);
				System.out.println("Error: " + error);
				System.out.println("Input: " + input);
				System.out.println();
			}
			
			if(error <= accepError) {
				System.out.println("Feet: " + pos/119.301);
				driveSpd(0.0, 0.0);
				break;
			}
		}
	}
	
	public void shuTurn(double goal, double percentError, double p, double max, boolean debug) {
		double feed_forward = 0.055;
		double accepError = goal/percentError;
		double angle = this.getAngle();
		double error = goal - angle; 
		double input = 0; 
		while(true) {
			angle = this.getAngle();
			error = goal - angle;
			input = error*p + feed_forward;
			if(input >= max)
				input = max;
			
			this.driveSpd(input, -input);
			
			if(debug) {
				System.out.println("Angle: " + angle);
				System.out.println("Error: " + error);
				System.out.println("Input: " + input);
				System.out.println();
			}
			
			if(error <= accepError) {
				System.out.println("Angle: " + angle);
				this.driveSpd(0.0, 0.0);
				break;
			}
		}
	}
	
	public void driveDist(double dist, double delay, boolean debug) {

		double feed_forward = 0.11; // forward input to reduce the steady state error
		double maxD = 0.275; // used to clamp the max speed, to slow down the robot
		double previous_errorD = 0; // used to calculate the derivative value
		double integralD = 0; // used to carry the sum of the error
		double derivativeD = 0; // used to calculate the change between our goal and position
		double KpD = 0.00419; // proportional constant
		double KiD = 0.0; // integral constant
		double KdD = 0;//0.002; // derivative constant
		double goal = dist*119.301;
		// this is what we want the robot to do: go forward,
		// turn, elevate, etc to a new position)
		double dt = delay;
		// this is the wait period for the loop e.g. 1/100s)
		double position = (this.getLeftEncoder() + this.getRightEncoder())/2;// current position in inches/feed, degrees, etc.)
		double offsetL =  (position - this.getLeftEncoder())/990.13; 
		double offsetR = (position - this.getRightEncoder())/990.13;
		double error = 0; // our goal is to make the error from the current position zero)
		double error_check = goal/100;
		
		while(true) {
			//Reset the Position
			position = this.getLeftEncoder();
			//Calculates the error based on how far the robot is from the dist
			error = goal - position;
			//Calculates the Integral based on the error, delay, and the previous integral 
			integralD = integralD + error * dt; 
			//Calculates the derivative based on the error and the delay 
			derivativeD = (error - previous_errorD) / dt;
			//MATH 
			double output = KpD * error + KiD * integralD + KdD * derivativeD + feed_forward; 
			//Passes on the error to the previous error
			previous_errorD = error;
			
			//NORMALIZE: If the spd is bigger than we want, set it to the max, if its less than the -max makes it the negitive max
			if(output > maxD)
				output = maxD;
			else if(output < -maxD)
				output = -maxD; 
			
			//After the spd has been fixed, set the speed to the output
			this.driveSpd(output+offset, output-offset);
			
			//If it's close enough, just break and end the loop 
			if(error <= error_check) {
				System.out.println("break");
				break;
			}
			
			//Delay(Uses dt)
			Timer.delay(dt);
			if(debug) {
				System.out.println("Position: " + position);
				System.out.println("Error: " + error);
				System.out.println("Output: " + output);
				System.out.println("Integral: " + integralD);
				
			}
		}
	}
	
	public void driveDoubleDist(double dist, double delay, boolean debug) {

		double feed_forward_1 = 0.11; 
		double feed_forward_2 = 0.125;// forward input to reduce the steady state error
		double max = 0.4; // used to clamp the max speed, to slow down the robot
		double previous_error_1 = 0;
		double previous_error_2 = 0;// used to calculate the derivative value
		double integral_1 = 0;
		double integral_2 = 0; // used to carry the sum of the error
		double derivative_1 = 0;
		double derivative_2 = 0;// used to calculate the change between our goal and position
		double Kp = 0.00419; // proportional constant
		double Ki = 0.002; // integral constant
		double Kd = 0.002; // derivative constant
		double goal = dist*119.301;
		// this is what we want the robot to do: go forward,
		// turn, elevate, etc to a new position)
		double dt = delay;
		// this is the wait period for the loop e.g. 1/100s)
		double position_1 = this.getLeftEncoder();
		double position_2 = this.getRightEncoder();// current position in inches/feed, degrees, etc.)
		double error_1 = 0;
		double error_2 = 0;// our goal is to make the error from the current position zero)
		double error_check = goal/1000;
		
		while(true) {
			//Reset the Position
			position_1 = this.getLeftEncoder();
			position_2 = this.getRightEncoder();
			//Calculates the error based on how far the robot is from the dist
			error_1 = goal - position_1;
			error_2 = goal - position_2;
			//Calculates the Integral based on the error, delay, and the previous integral 
			integral_1 = integral_1 + error_1 * dt;
			integral_2 = integral_2 + error_2 * dt;
			//Calculates the derivative based on the error and the delay 
			derivative_1 = (error_1 - previous_error_1) / dt;
			derivative_2 = (error_2 - previous_error_2) / dt;
			//MATH 
			double output_1 = Kp * error_1 + Ki * integral_1 + Kd * derivative_1 + feed_forward_1;
			double output_2 = Kp * error_2 + Ki * integral_2 + Kd * derivative_2 + feed_forward_2; 

			//Passes on the error to the previous error
			previous_error_1 = error_1;
			previous_error_2 = error_2;
			
			//NORMALIZE: If the spd is bigger than we want, set it to the max, if its less than the -max makes it the negitive max
			if(output_1 > max)
				output_1 = max;
			else if(output_1 < -max)
				output_1 = -max; 
			
			if(output_2 > max)
				output_2 = max;
			else if(output_2 < -max)
				output_2 = -max; 
			
			//After the spd has been fixed, set the speed to the output
			this.driveSpd(output_1, output_2);
			
			//If it's close enough, just break and end the loop 
			if(error_1 <= error_check && error_2 <= error_check && derivative_1 <= error_check && derivative_2 <= error_check) {
				System.out.println("break");
				System.out.println("Feet Left: " + (position_1/119.301));
				System.out.println("Feet right: " + (position_2/119.301));
				driveSpd(0,0);
				break;
			}
			
			//Delay(Uses dt)
			Timer.delay(dt);
			if(debug) {
				System.out.println("Position: " + position_1);
				System.out.println("Error: " + error_1);
				System.out.println("Output: " + output_1);
				System.out.println("Integral: " + integral_1);
				System.out.println("Position: " + position_2);
				System.out.println("Error: " + error_2);
				System.out.println("Output: " + output_2);
				System.out.println("Integral: " + integral_2);
				
			}
		}
	}
	public void turnToAngle(double angl, double delay, boolean debug) {
		double feed_forward = 0; // forward input to reduce the steady state error
		double max = 0.275; // used to clamp the max speed, to slow down the robot
		double previous_error = 0; // used to calculate the derivative value
		double integral = 0; // used to carry the sum of the error
		double derivative = 0; // used to calculate the change between our goal and position
		double Kp = 0.005; // proportional constant
		double Ki = 0.002; // integral constant
		double Kd = 0;//0.002; // derivative constant
		double goal = angl;
		// this is what we want the robot to do: go forward,
		// turn, elevate, etc to a new position)
		double dt = delay;
		// this is the wait period for the loop e.g. 1/100s)
		double position = this.getAngle(); // current position in inches/feed, degrees, etc.)
		double error = 0; // our goal is to make the error from the current position zero)
		double error_check = goal/100;
		
		while(true) {
			//Reset the Position
			position = this.getAngle();
			//Calculates the error based on how far the robot is from the dist
			error = goal - position;
			//Calculates the Integral based on the error, delay, and the previous integral 
			integral = integral + error * dt; 
			//Calculates the derivative based on the error and the delay 
			derivative = (error - previous_error) / dt;
			//MATH 
			double output = Kp * error + Ki * integral + Kd * derivative + feed_forward; 
			//Passes on the error to the previous error
			previous_error = error;
			
			//NORMALIZE: If the spd is bigger than we want, set it to the max, if its less than the -max makes it the negitive max
			if(output > max)
				output = max;
			else if(output < -max)
				output = -max; 
			
			//After the spd has been fixed, set the speed to the output
			this.driveSpd(output, -output);
			
			//If it's close enough, just break and end the loop 
			if(error <= error_check) {
				System.out.println("break");
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
	
	public double[] driveLeftConcurrently(double error, double previousError, double dt, double max, double previousIntegral, boolean debugOn) {
		double[] returnArray = new double[2];//first is the error, second is the derivative
		double Kp = 0.00419; // proportional constant 0.00419
		double Ki = 0.002; // integral constant 0.002
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
		_rearLeft.set(ControlMode.PercentOutput, OI.normalize(output,-max,0,max));
		_frontLeft.set(ControlMode.PercentOutput, OI.normalize(output,-max,0,max));
		
		if(debugOn) {
			System.out.print("Drive Right:");
			System.out.print("	Error: " + error);
			System.out.print("	Integral: " + integral);
			System.out.print("	Derivative: " + derivative);
			System.out.println("	Output: " + output);
		}
		returnArray[0] = error;
		returnArray[1] = derivative;
		return returnArray;
	}
	
	public double[] driveRightConcurrently(double error, double previousError, double dt, double max, double previousIntegral, boolean debugOn) {
		double[] returnArray = new double[2];//first is the error, second is the derivative
		double Kp = 0.00419; // proportional constant 0.00419
		double Ki = 0.002; // integral constant 0.002
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
		_rearRight.set(ControlMode.PercentOutput, OI.normalize(output,-max,0,max));
		_frontRight.set(ControlMode.PercentOutput, OI.normalize(output,-max,0,max));
		
		if(debugOn) {
			System.out.print("Drive Left:");
			System.out.print("	Integral: " + integral);
			System.out.print("	Derivative: (" + error + "-" + previousError + ") / " + dt);
			System.out.println("	Output: " + output);
		}
		
		returnArray[0] = error;
		returnArray[1] = derivative;
		return returnArray;
	}
	
	public double[] turnConcurrently(double error, double previousError, double dt, double max, double previousIntegral) {
		double[] returnArray = new double[2];//first is the error, second is the derivative
		double Kp = 0.005; // proportional constant
		double Ki = 0.002; // integral constant
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
		this.driveSpd(output,-output);
		returnArray[0] = error;
		returnArray[1] = derivative;
		return returnArray;
	}
	//Sets both motors to the same speed
//	public void driveSpd(double spd) {
//		_frontLeft.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
//		_frontRight.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
//		_rearLeft.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
//		_rearRight.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
//	}
	
	//Sets the Left and Right motor to different speed values
	public void driveSpd(double lspd, double rspd) {
		_frontLeft.set(ControlMode.PercentOutput, OI.normalize(lspd,-1,0,1));
		_frontRight.set(ControlMode.PercentOutput, OI.normalize(rspd,-1,0,1));
		_rearLeft.set(ControlMode.PercentOutput, OI.normalize(lspd,-1,0,1));
		_rearRight.set(ControlMode.PercentOutput, OI.normalize(rspd,-1,0,1));
	}
	
	public void driveSpdTime(double spd, double sec) {
		this.reset();
		Timer clock = new Timer();
		clock.start();
		
		_frontLeft.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
		_frontRight.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
		_rearLeft.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
		_rearRight.set(ControlMode.PercentOutput, OI.normalize(spd,-1,0,1));
		
		Timer.delay(sec);
		
		_frontLeft.set(ControlMode.PercentOutput, 0);
		_frontRight.set(ControlMode.PercentOutput, 0);
		_rearLeft.set(ControlMode.PercentOutput, 0);
		_rearRight.set(ControlMode.PercentOutput, 0);
		
		Double encoderTicks = (this.getLeftEncoder() + this.getRightEncoder())/2;
		System.out.println("Encoder: " + encoderTicks);
	}

	
//	public void driveDist(double distance, double speed, Elevator elevator, double time) {
//		reset();
//		Timer t = new Timer();
//		t.start();
//		int timeOut = 10;
//		if(distance < 500) {
//			timeOut = 5;
//		}
//		while ( Math.abs( ( getLeftEncoder() + getRightEncoder() )/2 ) <= distance && t.get() < timeOut) {
//			if (t.get() < .5) {
//				tankDrive(-0.005, speed * (t.get() * 2));
//			} else if (distance - _leftEncoder.getDistance() < 100) {
//				tankDrive(0, speed / 2);
//			} else {
//				tankDrive(-0.005, speed);
//			}
//			
//			if(t.get() < time && elevator.getLimit()) {
//				elevator.setElevator(0.8);
//			}else {
//				elevator.setElevator(0.0);
//			}
//			
////			System.out.println("Encoder: " + (( _leftEncoder.getDistance() + _rightEncoder.getDistance())/2));
//		}
//		tankDrive(0, 0);
//		while(t.get() < time && elevator.getLimit()) {
//			elevator.setElevator(0.8);
//		}
//		elevator.setElevator(0.0);
//		System.out.println("Left Encoder: " + getLeftEncoder());
//		System.out.println("Right Encoder: " + getRightEncoder());
//		System.out.println("Encoder: " + (( getLeftEncoder() + getRightEncoder())/2));
//	}
	
//	public void turnDist(double distance, double  speed) {
//		reset();
//		double startLeft = getLeftEncoder();
//		double startRight = getRightEncoder();
////		System.out.println("Init Left" + startLeft + "      InitRight: " + startRight);
//		Timer t = new Timer();
//		t.start();
//		while ( Math.abs( (((getLeftEncoder() - startLeft)-( getRightEncoder() -startRight))/2) ) <= distance && t.get() < 2.5) {
//				turn( -speed );
////			System.out.println("Encoder: " + (( _leftEncoder.getDistance() + _rightEncoder.getDistance())/2));
//		}
//		tankDrive(0, 0);
//		
//		System.out.println("Left Encoder:  " + getLeftEncoder());
//		System.out.println("Right Encoder: " + getRightEncoder());
//		System.out.println("Avg Encoder: " + Math.abs( (( getLeftEncoder() - getRightEncoder() )/2) ));
//		System.out.println("Gyro: " + this.getAngle());
//		System.out.println();
//	}
	
	//Gets instance for chassis 
	public static Chassis getInstance() {
		if (_instance == null) {
			_instance = new Chassis();
		}
		return _instance;
	}
	//Checks Encoder, and returns it 
	public double getLeftEncoder() {
		return _leftEncoder.getDistance();
	}
	
	//URGENT REMINDER: RIGHT ENCODER RETURNS NEGATIVE VALUES ON COMP ROBOT - CHANGE BEFORE COMP
	public double getRightEncoder() {
		return -1.0 * _rightEncoder.getDistance();
	}
	//Resets encoder value
	public void reset() {
		_leftEncoder.reset();
		_rightEncoder.reset();
		_gyro.reset();
	}
	//Checks and returns angle 
	public double getAngle() {
		return _gyro.getAngle();
	}

	//	public double getUltraValue() {
	//		return _ultra.getVoltage();
	//	} 

	//	public double returnDistance() {
	//		_ultra
	//	}

	//Stops the Robot
	public void stop() {
		tankDrive(0, 0);
	}


	//Uses MATH to make us turn good 
	//@param Angle we want, P constant
	//Sets intake Spd
	//Checks speed while turning, Corrects for over 1.0
	/*public void turnToAngle(double goal, double p, double accepErrorPercent) {
		double accepError = goal * accepErrorPercent / 100;//Translates to percentage
		Timer clock = new Timer();
		double angle;
		double error;
		double input;
		while(true) {
			angle = _gyro.getAngle();
			error = goal - angle;
			input = error * p;
			System.out.print("Goal: " + goal + " ");
			System.out.print("Angle: " + angle + " ");
			System.out.print("Error: " + error + " ");
			System.out.print("P: " + p + " ");
			System.out.println("Input: " + input + " ");
			shuTurn(-input);
			Timer.delay(0.1);
			if(error <= accepError) {
				clock.start();
				if(clock.hasPeriodPassed(.5)) {
					break;
				}
			}else if(error > accepError) {
				clock.stop();
			}
		}
		clock.stop();
		clock.reset();
	}
	*/
	
	public void turnToAngleLimePID() {
		turnToAngle(limelight.getTX(), 0.001, false);
	}

	public void simpleLimeTurn(double minAngleError){
		System.out.println(limelight.getTargetAngle());
		/*
		System.out.println("Turning with TX of: " + getTX());
		int count = 0;
		while(count<1000){
			Timer.delay(0.01);

			double absTX = (limelight.getTX())/30;
			//driveSpd(limelight.getTX()<0?-absTX:absTX,limelight.getTX()>0?-absTX:absTX);
			driveSpd(absTX,-absTX);
			count++;
			System.out.println("absTx: " + absTX + ", tx:" + limelight.getTX());
		}
		//driveSpd(0,0);*/
	}

	public void holdLimeTurn(){
		double absTX = limelight.getTX()/30;
		driveSpd(absTX, -absTX);
		driveSpd(0,0);
	}

	public double getTX(){
		return limelight.getTX();
	}
}

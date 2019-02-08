package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Driver {

	Chassis chassis;
	OI oi;
	Elevator elevator;
	Intake intake;
	Climber climber;
	String print;
	Port port = Port.kOnboard;
	
	public Driver() {
		chassis = Chassis.getInstance();
		oi = new OI();
		elevator = new Elevator();
		intake = new Intake();
		climber = new Climber();
	}
	
	public void reset() {
		chassis.reset();
		climber.reset();
	}
	
	public void driveRaise(double distanceGoal, double raiseGoal, boolean debugOn) {
		this.reset();
		Timer clock = new Timer();
		clock.start();
		double maxSpd = .25;
		double dt = 0.5;
		double previousLeftDriveError = distanceGoal;
		double previousRightDriveError = distanceGoal;
		double previousRaiseError = raiseGoal;
		double previousLeftIntegral = 0.001;
		double previousRightIntegral = 0.001;
		double previousRaiseIntegral = 0.001;
		double acceptDistError = .05;
		double acceptRaiseError = .05;
		//double leftError = distanceGoal*119.301 - chassis.getLeftEncoder();
		//double rightError = distanceGoal*119.301 - chassis.getRightEncoder();
		double previousRaiseDerivative = (raiseGoal*115.116 - climber.getDistance() - previousRaiseError) / dt;
		double previousRightDerivative = (distanceGoal*119.301 - chassis.getRightEncoder() - previousRightDriveError) / dt;
		//double origRDeriv = previousRightDerivative;
		double previousLeftDerivative = (distanceGoal*119.301 - chassis.getLeftEncoder() - previousLeftDriveError) / dt;
		//double origLDeriv = previousLeftDerivative;
		while(true) {
			double[] leftArray = chassis.driveLeftConcurrently((distanceGoal*119.301)-chassis.getLeftEncoder(), previousLeftDriveError, dt, maxSpd, previousLeftIntegral, debugOn);
			double[] rightArray = chassis.driveRightConcurrently((distanceGoal*119.301)-chassis.getLeftEncoder(), previousLeftDriveError, dt, maxSpd, previousLeftIntegral, debugOn);
			previousRightDerivative = rightArray[1];
			previousLeftDerivative = leftArray[1];
			previousLeftDriveError = leftArray[0];
			previousRightDriveError = rightArray[0];
			previousRaiseError = climber.raiseConcurrently((raiseGoal*115.116)-climber.getDistance(), previousRaiseError, dt, maxSpd, previousRaiseIntegral, debugOn);
			previousLeftIntegral = previousLeftIntegral + previousLeftDriveError * dt;
			previousRightIntegral = previousRightIntegral + previousRightDriveError * dt;
			previousRaiseIntegral = previousRaiseIntegral + previousRaiseError * dt;
			//leftError = distanceGoal*119.301 - chassis.getLeftEncoder();
			//rightError = distanceGoal*119.301 - chassis.getRightEncoder();
			//System.out.println("Right Encoder: " + chassis.getRightEncoder() + ". Right Drive Error " + previousRightDriveError);
			//System.out.println("Dt: " + dt);
			Timer.delay(dt);
			
//			System.out.println("previousRaiseError/100: " + previousRaiseError/100);
//			System.out.println("previousLeftDriveError/100: " + previousLeftDriveError/100);
//			System.out.println("previousRightDriveError/100: " + previousRightDriveError/100);
			
		//	System.out.println("Raise: " + previousRaiseError);
			System.out.println("Rightderive: " + previousRightDerivative);
			System.out.println("Leftderive: " + previousLeftDerivative);
			if(Math.abs(previousRaiseDerivative) <= 0.02 && Math.abs(previousRightDerivative) <= 0.02 && Math.abs(previousLeftDerivative) <= 0.02 && Math.abs(previousRaiseError/100) <= acceptRaiseError && Math.abs(previousRightDriveError/100) <= acceptDistError  && Math.abs(previousLeftDriveError/100) <= acceptDistError) {
				System.out.println("Breaking! :D");
				chassis.driveSpd(0,0);
				break;
			}
		}
		System.out.println("Distance Goal: " + distanceGoal);
		//System.out.println("Original Right Encoder: " + geetEncooder);
		/*System.out.println("Cool beans " + origRDeriv);
		System.out.println("Brawl Stars is the best " +origLDeriv);*/
		
		
		
	}
	
	public void turnRaise(double turnGoal, double raiseGoal, boolean debugOn) {
		
		this.reset();
		Timer clock = new Timer();
		clock.start();
		double maxSpd = .5;
		double dt = 0.01;
		double previousTurnError = turnGoal;
		double previousRaiseError = raiseGoal;
		double previousLeftIntegral = 0.001;
		double previousRaiseIntegral = 0.001;
		double turnError = turnGoal - chassis.getAngle();
		double previousTurnDerivative = (turnError - previousTurnError) / dt;
		while(true) {
			previousTurnError = chassis.turnConcurrently((turnGoal)-chassis.getAngle(), previousTurnError, dt, maxSpd, previousLeftIntegral)[0];
			previousRaiseError = climber.raiseConcurrently((raiseGoal*115.116)-climber.getDistance(), previousRaiseError, dt, maxSpd, previousRaiseIntegral, debugOn);
			previousLeftIntegral = previousLeftIntegral + previousTurnError * dt;
			previousRaiseIntegral = previousRaiseIntegral + previousRaiseError * dt;
			turnError = turnGoal - chassis.getAngle();
			previousTurnDerivative = chassis.turnConcurrently((turnGoal)-chassis.getAngle(), previousTurnError, dt, maxSpd, previousLeftIntegral)[1];
			Timer.delay(dt);
			if(previousTurnError <= turnError && Math.abs(previousTurnDerivative) <= 0.02)
				break;
		}
		
		
	}
}

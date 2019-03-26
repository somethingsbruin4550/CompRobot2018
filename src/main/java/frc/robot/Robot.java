package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.sql.Time;

import javax.swing.text.html.HTMLDocument.HTMLReader.BlockAction;

import frc.pixycam.PixyI2C;
import frc.pixycam.PixyUtil;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
// Class

public class Robot extends TimedRobot {

	
	
	//Dashboard Setup:
    
	boolean autoRun = false;
	public String switLoc;
	final String defaultAuto = "Default"; // should capitalize
	final String FORWARD = "Forward Cross";
	final String LEFT = "Left Position";
	final String RIGHT = "Right Postion";
	final String CENTER = "Center Postition";
	final String EXCHANGE_CENTER = "Exchange Center";
	final String EXCHANGE_LEFT = "Exchange Left";
	final String LEFT_SCALE_PRIO = "Left Scale - Priority";
	final String RIGHT_SCALE_PRIO = "Right Scale - Priority";
	final String LEFT_SCALE_ONLY = "Left Scale Only";
	final String RIGHT_SCALE_ONLY = "Right Scale Only";
	final String PIXY_TEST = "Pixy Test Code";

	String autoSelected;
	private SendableChooser<String> _chooser;

	Driver _driver;
	PixyI2C cam = new PixyI2C((byte) 0x54);
	boolean camSwitch = false;

	RevLED led = new RevLED();

//	Chassis _chassis;
//	OI _oi;
//	Elevator _elevator;
//	Intake _intake;
//	Climber _climber;

	UsbCamera _camera1;
	UsbCamera _camera2;
	CameraServer server;

	double spdMult = 1.0;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	// Runs at Startup
	public void robotInit() {
		_chooser = new SendableChooser<String>();
		_chooser.addDefault("Default Auto", defaultAuto);
		_chooser.addObject("Center", CENTER);
		_chooser.addObject("Left", LEFT);
		_chooser.addObject("Right", RIGHT);
		_chooser.addObject("Forward", FORWARD);
		_chooser.addObject("Exchange Center", EXCHANGE_CENTER);
		_chooser.addObject("Exchange Left", EXCHANGE_LEFT);
		_chooser.addObject("Left - Scale prio", LEFT_SCALE_PRIO);
		_chooser.addObject("Right - Scale prio", RIGHT_SCALE_PRIO);
		_chooser.addObject("Left - Scale ONLY", LEFT_SCALE_ONLY);
		_chooser.addObject("Right - Scale ONLY", RIGHT_SCALE_ONLY);
		_chooser.addObject("Pixy Camera Test (!WIP!)", PIXY_TEST);
		SmartDashboard.putData("Auto choices", _chooser);

		_driver = new Driver();
		_driver.reset();	

		//led.initLED();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	// Initializes Autonomous

	public void autonomousInit() {
//		switLoc = DriverStation.getInstance().getGameSpecificMessage();
		switLoc = "LRL";
		autoSelected = _chooser.getSelected();
		autoRun = true;
		_driver.reset();
		
		led.setDutyCycle(1435); //Sets Gray Breathe

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	// Runs Autonomous
	public void autonomousPeriodic() {
		double drvSpd = 0.4;
		double trnSpd = 0.8;
		double delay = 0;
		if (autoRun) {
			switch (autoSelected) {
//			case LEFT:
//				if (switLoc.charAt(0) == 'L') {
//					System.out.println("Left Left");
//					_chassis.driveDistNonTrapezoidal(300, drvSpd + .1);
//					Timer.delay(.5);
//					_chassis.driveDist(1100, drvSpd);
//					Timer.delay(.2);
//					_chassis.turnDist(175, trnSpd / 2);
//					_chassis.driveDist(200, drvSpd, _elevator, 2.0);
//					// _chassis.driveDist(50, drvSpd);
//
//					_intake.setIntake(-.3, -.3);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					_chassis.driveDistNonTrapezoidal(100, -drvSpd);
//					_chassis.turnDist(165, -trnSpd / 2);
//					_elevator.moveTime(-0.8, 1.5);
//				} else {
//					// Left side - change for later
//					System.out.println("Left Right");
////					if (switLoc.charAt(1) == 'L') {
////						_chassis.driveDistNonTrapezoidal(400, drvSpd + 0.1);
////						Timer.delay(0.5);
////						_chassis.driveDist(2150, drvSpd, _elevator, 4.500);
////						Timer.delay(0.2);
////						_chassis.turnDist(145, trnSpd / 2);
////						_chassis.driveDistNonTrapezoidal(50, 0.2);
////
////						_intake.setIntake(-.35, -.35);
////						Timer.delay(1.5);
////						_intake.setIntake(0, 0);
////						_chassis.turnDist(190, trnSpd / 2);
////						_elevator.moveTime(-0.8, 3.5);
////					} else {
//						_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//						Timer.delay(.5);
//						_chassis.driveDist(1200, drvSpd);
////					}
//				}
//				break;
//			case RIGHT:
//				if (switLoc.charAt(0) == 'L') {
//					System.out.println("Right Left");
////					if (switLoc.charAt(1) == 'R') {
////						_chassis.driveDistNonTrapezoidal(400, drvSpd + 0.1);
////						Timer.delay(0.5);
////						_chassis.driveDist(2150, drvSpd, _elevator, 4.50);
////						Timer.delay(0.2);
////						_chassis.turnDist(165, -trnSpd / 2);
////						_chassis.driveDistNonTrapezoidal(50, 0.2);
////
////						_intake.setIntake(-.35, -.35);
////						Timer.delay(1.5);
////						_intake.setIntake(0, 0);
////						_chassis.turnDist(190, -trnSpd / 2);
////						_elevator.moveTime(-0.8, 3.5);
////					} else {
//						_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//						Timer.delay(.5);
//						_chassis.driveDist(1200, drvSpd);
//				//	}
//				} else {
//					_chassis.driveDistNonTrapezoidal(300, drvSpd + 0.1);
//					Timer.delay(1.0);
//					_chassis.driveDist(1100, drvSpd, _elevator, 2.00);
//					Timer.delay(.2);
//					_chassis.turnDist(165, -trnSpd / 2);
//					_chassis.driveDist(100, drvSpd);
//					_intake.setIntake(-.25, -.25);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					_chassis.driveDistNonTrapezoidal(100, -drvSpd);
//					_chassis.turnDist(165, trnSpd / 2);
//					_elevator.moveTime(-0.8, 1.5);
//
//				}
//				break;
//			case CENTER:
//				if (switLoc.charAt(0) == 'L') {
//					System.out.println("Center Left");
//					_chassis.driveDist(400, drvSpd);
//					_chassis.turnDist(165, -trnSpd / 2);
//
//					_chassis.driveDistNonTrapezoidal(550, drvSpd);
//
//					_chassis.turnDist(100, trnSpd / 2);
//					_elevator.moveTime(0.8, .5);
//					Timer.delay(0.2);
//					_chassis.driveDist(400, drvSpd, _elevator, 1.5);
//
//					_intake.setIntake(-.25, -.25);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//
//				} else {
//					System.out.println("Center Right");
//					_chassis.driveDist(400, drvSpd);
//
//					_chassis.turnDist(100, trnSpd / 2);
//
//					_chassis.driveDistNonTrapezoidal(375, drvSpd);
//
//					_chassis.turnDist(90, -trnSpd / 2);
//					Timer.delay(.2);
//					_elevator.moveTime(0.8, .5);
//					_chassis.driveDist(450, drvSpd, _elevator, 1.5);
//
//					_intake.setIntake(-.25, -.25);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//
//				}
//				break;
//			case FORWARD:
//				System.out.println("Forward Cross");
//				_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//				Timer.delay(1.0);
//				_chassis.driveDist(1200, drvSpd);
//				break;
//			case EXCHANGE_CENTER:
//				System.out.println("Exchange Center");
//				_chassis.driveDistNonTrapezoidal(100, drvSpd);
//				_chassis.turnDist(165, -trnSpd / 2);
//				_chassis.driveDist(170, drvSpd);
//				_chassis.turnDist(165, -trnSpd / 2);
//				_chassis.driveDist(100, drvSpd);
//				_intake.setIntake(-.25, -.25);
//				Timer.delay(1.0);
//				_intake.setIntake(0, 0);
//				_chassis.turnDist(50, -trnSpd / 2);
//				_chassis.driveDistNonTrapezoidal(585, -drvSpd);
//
//				break;
//			case EXCHANGE_LEFT:
//				_chassis.driveDistNonTrapezoidal(100, drvSpd);
//				_chassis.turnDist(180, trnSpd / 2);
//				_chassis.driveDist(220, drvSpd);
//				_chassis.turnDist(180, trnSpd / 2);
//				_chassis.driveDist(100, drvSpd);
//				_intake.setIntake(-1, -1);
//				Timer.delay(1.0);
//				_intake.setIntake(0, 0);
//				_chassis.turnDist(35, -trnSpd / 2);
//				_chassis.driveDistNonTrapezoidal(800, -drvSpd);
//				break;
//			case LEFT_SCALE_PRIO:
//				if (switLoc.charAt(1) == 'L') {
//					_chassis.driveDistNonTrapezoidal(400, drvSpd + 0.1);
//					Timer.delay(0.5);
//					_chassis.driveDist(2150, drvSpd, _elevator, 4.50);
//					Timer.delay(0.2);
//					_chassis.turnDist(145, trnSpd / 2);
//					_chassis.driveDistNonTrapezoidal(50, 0.2);
//
//					_intake.setIntake(-.35, -.35);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					Timer.delay(.2);
//					_chassis.turnDist(190, trnSpd / 2);
//					// _chassis.driveDistNonTrapezoidal(100, -0.3);
//					_elevator.moveTime(-0.8, 3.5);
//				} else {
//					// Left side - change for later
//					if (switLoc.charAt(0) == 'L') {
//						System.out.println("Left Left");
//						_chassis.driveDistNonTrapezoidal(300, drvSpd + .1);
//						Timer.delay(.5);
//						_chassis.driveDist(1100, drvSpd);
//						Timer.delay(.2);
//						_chassis.turnDist(175, trnSpd / 2);
//						_chassis.driveDist(200, drvSpd, _elevator, 2.0);
//						// _chassis.driveDist(50, drvSpd);
//
//						_intake.setIntake(-.3, -.3);
//						Timer.delay(1.5);
//						_intake.setIntake(0, 0);
//						_chassis.driveDistNonTrapezoidal(100, -drvSpd);
//						_chassis.turnDist(165, -trnSpd / 2);
//						_elevator.moveTime(-0.8, 1.5);
//					} else {
//						_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//						Timer.delay(.5);
//						_chassis.driveDist(1200, drvSpd);
//					}
//				}
//				break;
//			case RIGHT_SCALE_PRIO:
//				if (switLoc.charAt(1) == 'R') {
//					_chassis.driveDistNonTrapezoidal(400, drvSpd + 0.1);
//					Timer.delay(1.0);
//					_chassis.driveDist(2150, drvSpd, _elevator, 4.5);
//					Timer.delay(0.2);
//					_chassis.turnDist(145, -trnSpd / 2);
//					_chassis.driveDistNonTrapezoidal(50, 0.2);
//
//					_intake.setIntake(-.35, -.35);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					_chassis.turnDist(190, -trnSpd / 2);
//					_elevator.moveTime(-.8, 3.5);
//
//				} else if (switLoc.charAt(0) == 'R') {
//					_chassis.driveDistNonTrapezoidal(300, drvSpd + 0.1);
//					Timer.delay(1.0);
//					_chassis.driveDist(1100, drvSpd, _elevator, 2.00);
//					Timer.delay(.2);
//					_chassis.turnDist(165, -trnSpd / 2);
//					_chassis.driveDist(100, drvSpd);
//					_intake.setIntake(-.25, -.25);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					_chassis.driveDistNonTrapezoidal(100, -drvSpd);
//					_chassis.turnDist(165, trnSpd / 2);
//					_elevator.moveTime(-0.8, 1.5);
//				} else {
//					_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//					Timer.delay(.5);
//					_chassis.driveDist(1200, drvSpd);
//				}
//				break;
//			case LEFT_SCALE_ONLY:
//				if (switLoc.charAt(1) == 'L') {
//					_chassis.driveDistNonTrapezoidal(400, drvSpd + 0.1);
//					Timer.delay(0.5);
//					_chassis.driveDist(2150, drvSpd, _elevator, 4.50);
//					Timer.delay(0.2);
//					_chassis.turnDist(145, trnSpd / 2);
//					_chassis.driveDistNonTrapezoidal(50, 0.2);
//
//					_intake.setIntake(-.35, -.35);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					Timer.delay(.2);
//					_chassis.turnDist(190, trnSpd / 2);
//					_elevator.moveTime(-0.8, 3.5);
//				} else {
//					_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//					Timer.delay(.5);
//					_chassis.driveDist(1200, drvSpd);
//				}
//				break;
//			case RIGHT_SCALE_ONLY:
//				if (switLoc.charAt(1) == 'R') {
//					_chassis.driveDistNonTrapezoidal(400, drvSpd + 0.1);
//					Timer.delay(1.0);
//					_chassis.driveDist(2150, drvSpd, _elevator, 4.5);
//					Timer.delay(0.2);
//					_chassis.turnDist(145, -trnSpd / 2);
//					_chassis.driveDistNonTrapezoidal(50, 0.2);
//
//					_intake.setIntake(-.35, -.35);
//					Timer.delay(1.5);
//					_intake.setIntake(0, 0);
//					_chassis.turnDist(190, -trnSpd / 2);
//					_elevator.moveTime(-.8, 3.5);
//				} else {
//					_chassis.driveDistNonTrapezoidal(200, drvSpd + 0.1);
//					Timer.delay(.5);
//					_chassis.driveDist(1200, drvSpd);
//				}
//				break;
			case PIXY_TEST:
				System.out.println("PIXY_TEST Running");
				
				cam.setLamp(false, false);
				Timer.delay(0.5);
				cam.getGrayscale(50, 50, false);
				Timer.delay(0.5);
				boolean singleRun = false;
				int gval = cam.getGrayscale(50, 50, false);
				while(!_driver.oi.getR1() && !singleRun){
					int gvalTemp = cam.getGrayscale(50, 50, false);
					if(gval != gvalTemp) {
						//System.out.println("Grayscale: " + PixyUtil.printHex((byte) (gval >> 8)) + " " + PixyUtil.printHex((byte) (gval & 0xff)));
						//SmartDashboard.putString("Grayscale: ", (PixyUtil.printHex((byte) (gval >> 8)) + " " + PixyUtil.printHex((byte) (gval & 0xff))));
						//SmartDashboard.putString("iS It bRiGHt??", (PixyUtil.printHex((byte) (gval >> 8)).equals("0xff") ? "0xff" : PixyUtil.printHex((byte) (gval & 0xff))))	;
						
						gval = gvalTemp;
					}
					Timer.delay(0.1);
				}
				SmartDashboard.putString("Grayscale: ", "Ended");
				System.out.println("Exiting Pixy Test");
				

				/*
				int[] resolution = cam.getResolution();
				System.out.println("Resolution:");
				System.out.println("X: " + resolution[0]);
				System.out.println("Y: " + resolution[1]);
				*/
				/*
				Timer.delay(1);
				cam.setLamp(false, true);
				Timer.delay(1);
				cam.setLamp(true, false);
				Timer.delay(1);
				cam.setLamp(false, false);
				*/
				break;
				
			case defaultAuto:
				System.out.println("defaultAuto Running");
				_driver.chassis.simpleLimeTurn();
				/*
				_driver.chassis.limelight.setLED(true);
				Timer.delay(0.5);
				System.out.println("Original: " + _driver.chassis.limelight.getTX());
				_driver.chassis.getFinalLimelightAngle();
				_driver.chassis.limelight.setLED(false);*/
				/*
				while(!_driver.oi.getXButton()){
					
					for(int i = 500; i<1995; i+=1){
						led.setDutyCycle(i);
						Timer.delay(0.1);
					}
					
					int dutyCycle = 0;
					SmartDashboard.getNumber("LED DutyCycle", dutyCycle);
					led.setDutyCycle(dutyCycle);
				}*/
				//_driver.chassis.turnToAngleLimePID();
				break;
			}
		}
		autoRun = false;
	}

	public void teleopInit() {
		_driver.reset();
		//led.setDutyCycle(494); //Sets Blue Breathe
		int dutyCycle = (int)(SmartDashboard.getNumber("LED DutyCycle", 0.0));
		//int dutyCycle = SmartDashboard.getNumber("LED DutyCycle", dutyCycle);//SmartDashboard.getEntry("LED DutyCycle");
		led.setDutyCycle(dutyCycle);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	// Runs Teleop
	public void teleopPeriodic() {
			System.out.println("TX Value: "+ _driver.chassis.getTX());

			_driver.chassis.tankDrive(OI.normalize(Math.pow(_driver.oi.getRJoystickXAxis(), 3), -1.0, 0, 1.0) * 0.7,
					OI.normalize(Math.pow(_driver.oi.getLJoystickYAxis(), 3), -1.0, 0, 1.0) * 0.7);
			if (!_driver.elevator.getLimit()) {
				System.out.println("Pressed");
			}
			System.out.println("Left:" + _driver.chassis.getLeftEncoder());
			System.out.println("Right:" + _driver.chassis.getRightEncoder());
//		System.out.println("Gyro: " + _chassis.getAngle());
//		System.out.println("Input:" + _oi.getRJoystickXAxis());
			System.out.println("Climber" + _driver.climber.getDistance());
			// if(_oi.getXButton()) {
			// spdMult = 0.6;
			// }else if(_oi.getTriangleButton()) {
			// spdMult = 1.0;
			// }
			// This Allows for the chassis driver to slow down if necessary

			// System.out.println("Angle: " + _chassis.getAngle());
			// System.out.println("Right Encoder: " + _chassis.getRightEncoder());

			if (_driver.oi.getXButton()) {
				System.out.println("Left:" + _driver.chassis.getLeftEncoder());
				System.out.println("Right:" + _driver.chassis.getRightEncoder());
		//		System.out.println("Gyro: " + _chassis.getAngle());
				_driver.intake.setIntake(0.4, 0.65);
			} else if (_driver.oi.getOButton()) {
				_driver.intake.setIntake(-0.25, -0.25);
			} else if (_driver.oi.getTriangleButton()) {
				_driver.intake.setIntake(-0.5, -0.5);
			} else if (_driver.oi.getSquareButton()) {
				_driver.intake.setIntake(-1.0, -1.0);
			} else {
				_driver.intake.setIntake(0, 0);
			}

			if (_driver.oi.getR2() > 0.05 && _driver.elevator.getLimit()) {
				_driver.elevator.setElevator(Math.pow(_driver.oi.normalize(_driver.oi.getR2C2(), -1, 0, 1), 3));
			} else if (_driver.oi.getL2() > 0.05) {
				_driver.elevator.setElevator(Math.pow(_driver.oi.normalize(-1.0 * _driver.oi.getL2C2(), -1, 0, 1), 3));
			} else {
				_driver.elevator.setElevator(0);
			}
			
			if (_driver.oi.getR1()) {
				_driver.climber.setClimber(1.0);
			} else if (_driver.oi.getL1()) {
				_driver.climber.setClimber(-1.0);
			} else {
				_driver.climber.setClimber(0.0);
			}

			if(_driver.oi.getSelect()){
				System.out.println("Holding Lime Turn");
				_driver.chassis.holdLimeTurn();
			}
		}
	//}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

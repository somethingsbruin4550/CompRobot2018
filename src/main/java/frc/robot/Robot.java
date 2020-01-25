/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.parent.*;
import frc.sensors.*;

// import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.CCSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.trajectory.*;
import java.nio.file.Paths;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotMap, ControMap {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kResetPIDs = "Reset PIDs";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static int alliance;
  
  Trajectory trajectory;


  double spdmlt = 1.0;
  // Elevator elevator;
  CCSparkMax shooterMax;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    try{
      trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Unnamed.wpilib.json"));
    }catch(Exception e){
      //
    } 

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Reset PID Values", kResetPIDs);
    SmartDashboard.putData("Auto choices", m_chooser);
    // elevator = new Elevator();
    shooterMax = new CCSparkMax(10, MotorType.kBrushless, IdleMode.kBrake, false);
    LemonLight.setLightChannel(9);
  
    switch(DriverStation.getInstance().getAlliance()){
      case Blue:
        alliance = 1;
      break;

      case Red:
        alliance = 0; 
      break;
      
      case Invalid:
        alliance = -1;
      break;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);


    double Kp = SmartDashboard.getEntry("Kp").getDouble(0.0);
    System.out.println("Shooter Kp: " + Kp);
    double Ki = SmartDashboard.getEntry("Ki").getDouble(0.0);
    System.out.println("Shooter Ki: " + Ki);
    double Kd = SmartDashboard.getEntry("Kd").getDouble(0.0);
    System.out.println("Shooter Kd: " + Kd);
    
    shooterMax.setPID(Kp, Ki, Kd, 0);




    switch (m_autoSelected) {
      case kCustomAuto:
        shooterMax.setPosition(0);
        shooterMax.setReferencePosition(50);
        System.out.println("End Auto");
        // shooterMax.set
        break;
      case kDefaultAuto:
        shooterMax.setPosition(0);
        break;
      case kResetPIDs:
        SmartDashboard.putNumber("Kp", 0.0);
        SmartDashboard.putNumber("Ki", 0.0);
        SmartDashboard.putNumber("Kd", 0.0);
        break;
      default:
        
        break;
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    System.out.println(shooterMax.getPosition());
  }

  @Override
  public void teleopInit() {
    LemonLight.setLight(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    spdmlt = 13.619 * LemonLight.distToTarget() + 45;

    if(!OI.button(PilotMap.STICK_MID)){
      Chassis.axisDrive(OI.normalize(OI.axis(PilotMap.Y_AXIS), -1.0, 1.0), 
                          -OI.normalize(OI.axis(PilotMap.X_AXIS), -1.0, 1.0));
    } else {
      Chassis.lockOn(false, 3.0);
    }

    if(OI.button(PilotMap.STICK_BACK)){
      shooterMax.set(spdmlt);
    }
    else{
      shooterMax.disable();
    }

    // spdmlt = OI.normalize(OI.axis(PilotMap.Z_AXIS), 0, 1);

    System.out.println(spdmlt);
    // System.out.println(Chassis.distToTarget());

    // System.out.println(Chassis.distToTarget() + " Meters");
     // Chassis.followTarget();
      if(OI.button(PilotMap.STICK_LEFT))
        Intake.set(1.0);
      else if(OI.button(PilotMap.TRIGGER))
        Intake.set(-1.0);
      else
        Intake.set(0.0);

      // if(OI.button(PilotMap.STICK_MID))
      //   elevator.set(-1.0);
      // else if(OI.button(PilotMap.STICK_BACK))
      //   elevator.set(1.0);
      // else
      //   elevator.set(0.0);

    }

  @Override
  public void disabledInit() {
   LemonLight.setLight(false);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    LemonLight.setLight(true);

  }
}

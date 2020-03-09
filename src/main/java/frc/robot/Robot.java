/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Solenoid;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private DifferentialDrive m_myRobot;
  private XboxController dCon;
  private XboxController oCon;
  private static final int leftFDeviceID = 9; 
  private static final int leftBDeviceID = 7; 
  private static final int rightFDeviceID = 10;
  private static final int rightBDeviceID = 8;
  private static final int hoodMotorID = 11;
  private CANSparkMax m_leftFMotor;
  private CANSparkMax m_leftBMotor;
  private CANSparkMax m_rightFMotor;
  private CANSparkMax m_rightBMotor;
  private CANSparkMax hoodMotor;

  private VictorSPX IntakeMotor;
  private VictorSPX HopperMotor1;
  private VictorSPX HopperMotor2;
  private VictorSPX ShooterMotor;

  private Solenoid IntakeSol;
  
  private SpeedControllerGroup lMotorGroup; 
  private SpeedControllerGroup rMotorGroup; 
  
  private CANEncoder m_Lencoder;
  private CANEncoder m_Rencoder;
  private CANEncoder m_Sencoder;

  private Counter ShooterMag;
  private Counter ShooterIndex;
  private Counter HoodMag;
  private Counter TrenchMag;

  private Compressor MainComp;

  private Pixy2 pixy;

  private boolean FireMode;
  private int ShooterToggle;
  private int IntakeToggle;

  private double ShooterRevTarget = 0;
  private double ShooterRevCurrent = 0;
  private double AutonEncodeStart = 0;

  private double StoredTime = System.currentTimeMillis();


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override 
  public void robotInit() {
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_leftFMotor = new CANSparkMax(leftFDeviceID, MotorType.kBrushless);
    m_leftBMotor = new CANSparkMax(leftBDeviceID, MotorType.kBrushless);
    m_rightFMotor = new CANSparkMax(rightFDeviceID, MotorType.kBrushless);
    m_rightBMotor = new CANSparkMax(rightBDeviceID, MotorType.kBrushless);
    hoodMotor = new CANSparkMax(hoodMotorID, MotorType.kBrushless);

    m_Lencoder = m_leftFMotor.getEncoder();
    m_Rencoder = m_rightFMotor.getEncoder();
    m_Sencoder = hoodMotor.getEncoder();

    m_leftFMotor.restoreFactoryDefaults();
    m_leftBMotor.restoreFactoryDefaults();
    m_rightFMotor.restoreFactoryDefaults();
    m_rightBMotor.restoreFactoryDefaults();

    lMotorGroup = new SpeedControllerGroup(m_leftFMotor,m_leftBMotor);
    rMotorGroup = new SpeedControllerGroup(m_rightFMotor,m_rightBMotor);
    m_myRobot = new DifferentialDrive(lMotorGroup, rMotorGroup); 

    IntakeMotor = new VictorSPX(5);
    ShooterMotor = new VictorSPX(6);
    
    HopperMotor1 = new VictorSPX(3);
    HopperMotor2 = new VictorSPX(4);

    ShooterMag = new Counter(0); 
		ShooterIndex = new Counter(1);
    HoodMag = new Counter(2); 
    TrenchMag = new Counter(3);
    //Set Semi-Period Mode in order to Measure the Pulse Width
    ShooterMag.setSemiPeriodMode(true);
    HoodMag.setSemiPeriodMode(true);
    TrenchMag.setSemiPeriodMode(true);

    MainComp = new Compressor(2);
    IntakeSol = new Solenoid(2,1);
    //IntakeSol = new Solenoid(1);
    
    dCon = new XboxController(0);
    oCon = new XboxController(1);
    Pixy2 pixy = Pixy2.createInstance(new SPILink());
    pixy.init(); // Initializes the camera and prepares to send/receive data
		pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
    pixy.setLED(255, 255, 255); // Sets the RGB LED to green

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    FireMode = false;
    ShooterToggle = 2;
    IntakeToggle = 2;
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
    SmartDashboard.putNumber("Right Encoder Position", m_Rencoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Position", m_Lencoder.getPosition());
    SmartDashboard.putNumber("Shooter Position", m_Sencoder.getPosition());
    //SmartDashboard.putNumber("Trench Intermediate", TrenchMag.getPeriod());
    SmartDashboard.putBoolean("Compressor on", MainComp.getPressureSwitchValue());
    if (ShooterToggle == 1) {
      SmartDashboard.putString("Shooter State", "ENABLED");
    } else if (ShooterToggle == 0) {
      SmartDashboard.putString("Shooter State", "DISABLED");
    } else {
      SmartDashboard.putString("Shooter State", "???");
    }
    if (IntakeToggle == 1) {
      SmartDashboard.putString("Intake Status", "DOWN");
    } else if (IntakeToggle == 0) {
      SmartDashboard.putString("Intake Status", "UP");
    } else {
      SmartDashboard.putString("Intake Status", "???");
    }
    SmartDashboard.putNumber("Shooter Speed", ShooterRevCurrent);
    // The 9.73e-4 is the total period of the PWM output on the am-3749
		// The value will then be divided by the period to get duty cycle.
		// This is converted to degrees and Radians
		//double angleDEG = (value/9.739499999999999E-4)*361 -1;
		//double angleRAD = (value/9.739499999999999E-4)*2*(Math.PI) ;
		//SmartDashboard.putNumber("Angle in Degrees", angleDEG);
		//SmartDashboard.putNumber("Angle in Radians", angleRAD);
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
    AutonEncodeStart = m_Rencoder.getPosition(); 
    StoredTime = System.currentTimeMillis();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
      
      if (System.currentTimeMillis() - StoredTime < 3000) {
        ShooterMotor.set(ControlMode.PercentOutput, -1); 
        if (m_Rencoder.getPosition() - AutonEncodeStart > -20) {
          rMotorGroup.set(-.2);
          lMotorGroup.set(.2);
        } else { 
          rMotorGroup.set(0);
          lMotorGroup.set(0);
        }
      } else if (System.currentTimeMillis() - StoredTime < 7000) {
        HopperMotor1.set(ControlMode.PercentOutput,-0.15);
        HopperMotor2.set(ControlMode.PercentOutput,-0.15);
        rMotorGroup.set(0);
        lMotorGroup.set(0); 
        AutonEncodeStart = 0;
      } else if (System.currentTimeMillis() - StoredTime < 10000) {
        HopperMotor1.set(ControlMode.PercentOutput,0);
        HopperMotor2.set(ControlMode.PercentOutput,0);
        ShooterMotor.set(ControlMode.PercentOutput,0);  
        if (m_Rencoder.getPosition() - AutonEncodeStart < 36) {
          rMotorGroup.set(.4);
          lMotorGroup.set(-.4);
        } else { 
          rMotorGroup.set(0);
          lMotorGroup.set(0);
        }
      } else if (System.currentTimeMillis() - StoredTime < 15000) {
        rMotorGroup.set(0);
        lMotorGroup.set(0);
      }
     

      
      break;
      case kDefaultAuto:
      default:
      if (m_Rencoder.getPosition() - AutonEncodeStart > -40) {
        rMotorGroup.set(-.4);
        lMotorGroup.set(.4);
      } else {
        rMotorGroup.set(0);
        lMotorGroup.set(0);
      }
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-dCon.getY(Hand.kLeft), -dCon.getY(Hand.kRight));
    
    if (oCon.getBumperPressed(Hand.kRight)) { // Intake In
      IntakeMotor.set(ControlMode.PercentOutput, -0.2);
    } else if (oCon.getBumperReleased(Hand.kRight)) {
      IntakeMotor.set(ControlMode.PercentOutput, 0); 
    }
    if (oCon.getBumperPressed(Hand.kLeft)) { // Intake out
      IntakeMotor.set(ControlMode.PercentOutput, 0.2);
    } else if (oCon.getBumperReleased(Hand.kLeft)) {
      IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
    //
    if (oCon.getXButtonPressed()) { // Hopper In
      HopperMotor1.set(ControlMode.PercentOutput,-0.2);
      HopperMotor2.set(ControlMode.PercentOutput,-0.2);
    } else if (oCon.getXButtonReleased()) {
      HopperMotor1.set(ControlMode.PercentOutput, 0);
      HopperMotor2.set(ControlMode.PercentOutput, 0);
    }
    if (oCon.getYButtonPressed()) { // Hopper out
      HopperMotor1.set(ControlMode.PercentOutput,0.2);
      HopperMotor2.set(ControlMode.PercentOutput,0.2);
    } else if (oCon.getYButtonReleased()) {
      HopperMotor1.set(ControlMode.PercentOutput,0);
      HopperMotor2.set(ControlMode.PercentOutput,0);
    }
    //
    //if (m_Sencoder.getPosition() + (oCon.getY(Hand.kRight)/6) < -20 && m_Sencoder.getPosition() + (oCon.getY(Hand.kRight)/6) > 100) {
      //hoodMotor.set(dCon.getY(Hand.kLeft)/2); 
    //}  
    //
    if (oCon.getAButtonPressed() && ShooterToggle < 2) { // Shooter toggle
      if (ShooterToggle == 0) {
        ShooterToggle = 3;
      } else if (ShooterToggle == 1) {
        ShooterToggle = 2;
      }
    } else if (oCon.getAButtonReleased() && ShooterToggle > 1) {
      ShooterToggle = ShooterToggle - 2; 

      ShooterRevTarget = -ShooterToggle;
    }
    if (ShooterRevCurrent > ShooterRevTarget) {
      ShooterRevCurrent = ShooterRevCurrent - 0.02;
      ShooterMotor.set(ControlMode.PercentOutput, ShooterRevCurrent); 
    } else if (ShooterRevCurrent < ShooterRevTarget) {
      ShooterRevCurrent = ShooterRevCurrent + 0.02;
      ShooterMotor.set(ControlMode.PercentOutput, ShooterRevCurrent); 
    }
    //
    if (oCon.getBButtonPressed() && IntakeToggle < 2) { // Intake toggle
      if (IntakeToggle == 0) {
        IntakeToggle = 3;
      } else if (IntakeToggle == 1) {
        IntakeToggle = 2;
      }
    } else if (oCon.getBButtonReleased() && IntakeToggle > 1) {
      IntakeToggle = IntakeToggle - 2; 
      if (IntakeToggle == 1) {
        IntakeSol.set(true);
      } else if (IntakeToggle == 0) {
        IntakeSol.set(false);
      }
    }
    //
  }

  //@Override
  //public void testInit() {
    // TODO Auto-generated method stub
    //super.testInit();
    //pixy.setLamp((byte) 1, (byte) 1);
  //}
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (dCon.getAButtonPressed()) { //Intake Down
      ShooterMotor.set(ControlMode.PercentOutput,-1);
      
    } else if (dCon.getAButtonReleased()) {
      ShooterMotor.set(ControlMode.PercentOutput,0);
     // pixy.setLamp((byte) 0, (byte) 0);
    }
    
    HopperMotor1.set(ControlMode.PercentOutput,-dCon.getY(Hand.kRight)/4);
    HopperMotor2.set(ControlMode.PercentOutput,-dCon.getY(Hand.kRight)/4);
    IntakeMotor.set(ControlMode.PercentOutput, -dCon.getY(Hand.kRight)/4);
    hoodMotor.set(dCon.getY(Hand.kLeft)/6); 
  }
}

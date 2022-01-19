/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.DPad;
import org.usfirst.frc3620.misc.TriggerButton;
import org.usfirst.frc3620.misc.XBoxConstants;
import org.usfirst.frc3620.misc.CANDeviceType;


import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * 
 * @version 11 February 2020
 * 
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final static Logger logger = EventLogging.getLogger(RobotContainer.class, Level.INFO);
  final static int DRIVER_JOYSTICK_PORT = 0;
  final static int OPERATOR_JOYSTICK_PORT = 1;

  public final static double DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT = 0.3;
  public final static double AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT = 0.3;

  // need this
  static CANDeviceFinder canDeviceFinder;

  // hardware here...
  public static CANSparkMax driveSubsystemRightFrontDrive;
  public static CANSparkMax driveSubsystemRightFrontAzimuth;
  public static RelativeEncoder driveSubsystemRightFrontDriveEncoder;
  public static RelativeEncoder driveSubsystemRightFrontAzimuthEncoder;
  public static AnalogInput driveSubsystemRightFrontHomeEncoder;
  
  public static CANSparkMax driveSubsystemLeftFrontDrive;
  public static CANSparkMax driveSubsystemLeftFrontAzimuth;
  public static RelativeEncoder driveSubsystemLeftFrontDriveEncoder;
  public static RelativeEncoder driveSubsystemLeftFrontAzimuthEncoder;
  public static AnalogInput driveSubsystemLeftFrontHomeEncoder;
  
  public static CANSparkMax driveSubsystemLeftBackDrive;
  public static CANSparkMax driveSubsystemLeftBackAzimuth;
  public static RelativeEncoder driveSubsystemLeftBackDriveEncoder;
  public static RelativeEncoder driveSubsystemLeftBackAzimuthEncoder;
  public static AnalogInput driveSubsystemLeftBackHomeEncoder;
  
  public static CANSparkMax driveSubsystemRightBackDrive;
  public static CANSparkMax driveSubsystemRightBackAzimuth;
  public static RelativeEncoder driveSubsystemRightBackDriveEncoder;
  public static RelativeEncoder driveSubsystemRightBackAzimuthEncoder;
  public static AnalogInput driveSubsystemRightBackHomeEncoder;

  private static DigitalInput practiceBotJumper;

  public static Compressor theCompressor;

  // subsystems here...
  public static DriveSubsystem driveSubsystem;

  // joysticks here....
  public static Joystick driverJoystick;
  public static Joystick operatorJoystick;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();
    logger.info ("CAN bus: " + canDeviceFinder.getDeviceSet());

    identifyRoboRIO();

    makeHardware();
    setupMotors();
    makeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
    setupSmartDashboardCommands();
  }

  void setupMotors() {
    int kTimeoutMs = 0;

    if (driveSubsystemRightFrontDrive != null){

      resetMaxToKnownState(driveSubsystemRightFrontDrive);
      driveSubsystemRightFrontDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemRightFrontAzimuth);
      driveSubsystemRightFrontAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftFrontDrive);
      driveSubsystemLeftFrontDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftFrontAzimuth);
      driveSubsystemLeftFrontAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftBackDrive);
      driveSubsystemLeftBackDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftBackAzimuth);
      driveSubsystemLeftBackAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemRightBackDrive);
      driveSubsystemRightBackDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);
      
      resetMaxToKnownState(driveSubsystemRightBackAzimuth);
      driveSubsystemRightBackAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);
    }
  }

  void makeHardware() {
    boolean iAmACompetitionRobot = amIACompBot();
    //if (!iAmACompetitionRobot) {
    //  logger.warn ("this is a test chassis, will try to deal with missing hardware!");
    //}

    // we don't need to use the canDeviceFinder for CAN Talons because
    // they do not put up unreasonable amounts of SPAM

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 1, "Swerve") || iAmACompetitionRobot){

      driveSubsystemRightFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
      driveSubsystemRightFrontDriveEncoder = driveSubsystemRightFrontDrive.getEncoder();

      driveSubsystemRightFrontAzimuth = new CANSparkMax(2, MotorType.kBrushless);
      driveSubsystemRightFrontAzimuthEncoder = driveSubsystemRightFrontAzimuth.getEncoder();

      driveSubsystemRightFrontHomeEncoder = new AnalogInput(0);
              
      driveSubsystemLeftFrontDrive = new CANSparkMax(3, MotorType.kBrushless);
      driveSubsystemLeftFrontDriveEncoder = driveSubsystemLeftFrontDrive.getEncoder();
              
      driveSubsystemLeftFrontAzimuth = new CANSparkMax(4, MotorType.kBrushless);
      driveSubsystemLeftFrontAzimuthEncoder = driveSubsystemLeftFrontAzimuth.getEncoder();

      driveSubsystemLeftFrontHomeEncoder = new AnalogInput(1);
      
      driveSubsystemLeftBackDrive = new CANSparkMax(5, MotorType.kBrushless);
      driveSubsystemLeftBackDriveEncoder = driveSubsystemLeftBackDrive.getEncoder();
              
      driveSubsystemLeftBackAzimuth = new CANSparkMax(6, MotorType.kBrushless);
      driveSubsystemLeftBackAzimuthEncoder = driveSubsystemLeftBackAzimuth.getEncoder();

      driveSubsystemLeftBackHomeEncoder = new AnalogInput(2);
              
      driveSubsystemRightBackDrive = new CANSparkMax(7, MotorType.kBrushless);
      driveSubsystemRightBackDriveEncoder = driveSubsystemRightBackDrive.getEncoder();
      
      driveSubsystemRightBackAzimuth = new CANSparkMax(8, MotorType.kBrushless);
      driveSubsystemRightBackAzimuthEncoder = driveSubsystemRightBackAzimuth.getEncoder();

      driveSubsystemRightBackHomeEncoder = new AnalogInput(3);
    }
  }

  void setupSmartDashboardCommands() {
    SmartDashboard.putData(new ZeroDriveEncodersCommand(driveSubsystem));
  
    SmartDashboard.putData("TestAuto", new TestAuto(driveSubsystem));
    SmartDashboard.putData("5 Ball Auto", new FiveBallAuto(driveSubsystem));
  }

  void makeSubsystems() {
    driveSubsystem = new DriveSubsystem();
  }
  static void resetMaxToKnownState(CANSparkMax x) {
    x.setInverted(false);
    x.setIdleMode(IdleMode.kCoast);
    x.setOpenLoopRampRate(1);
    x.setClosedLoopRampRate(1);
    x.setSmartCurrentLimit(50);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  driverJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
  }

  public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y);
    SmartDashboard.putNumber("driver.raw.y", axisValue);
    if (axisValue < 0.15 && axisValue > -0.15) {
      return 0;
    }
    if (axisValue < 0){
      return (axisValue*axisValue);
    }
    return -axisValue*axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X);
    SmartDashboard.putNumber("driver.raw.x", axisValue);
    if (axisValue < 0.15 && axisValue > -0.15) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }

  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X);
    SmartDashboard.putNumber("driverSpin", axisValue);
    if (axisValue < 0.2 && axisValue > -0.2) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }
    
  public static double getOperatorSpinJoystick() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X);
    if (axisValue < 0.15 && axisValue > -0.15) {
      return 0;
    }
    return -axisValue;
  }

  /**
   * Return the climbing joystick position. Return positive values if
   * the operator pushes the joystick up.
   * @return
   */

  /**
   * Determine if this robot is a completition robot. It is if
   * it's connected to an FMS.
   * 
   * We should probably also check for an "I am a test" file or jumper
   * and return true if those are missing.
   * 
   * @return true if this robot is a competition robot.
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new GoldenAutoCommand(driveSubsystem, shooterSubsystem, visionSubsystem, intakeSubsystem);
    return null;
  }
   
  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    /*
    # GCF - Removing this condition

    if(practiceBotJumper.get() == true){
      return true;
    }
    */


    return false;
  }
  

  
  public static String identifyRoboRIO() {
    String rv = "";
    try {
			for (Enumeration<NetworkInterface> e = NetworkInterface
					.getNetworkInterfaces(); e.hasMoreElements();) {
				NetworkInterface network = e.nextElement();
				logger.info("found network {}", network);
				byte[] mac = network.getHardwareAddress();
				if (mac != null) {
					StringBuilder sb = new StringBuilder();
					for (int i = 0; i < mac.length; i++) {
						sb.append(String.format("%02X%s", mac[i],
								(i < mac.length - 1) ? "-" : ""));
					}
					String macString = sb.toString();
          logger.info("Current MAC address: {}", macString);
          if (network.getName().equals("eth0")) {
            rv = macString;
          }
				}
			}
		} catch (SocketException e) {
			e.printStackTrace();
    }
    return rv;
  }
}

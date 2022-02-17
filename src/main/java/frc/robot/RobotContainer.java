/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.AnalogJoystickButton;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.DPad;
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
  private static DigitalInput practiceBotJumper;

  // drive subsystem hardware
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

  //intake
  public static CANSparkMax intakeWheelbar;
  public static CANSparkMax intakeBelt;

  // vision
  private static Solenoid ringLight;

  // shooter hardware verables are currently unknown so we need to change them
  public static WPI_TalonFX shooterSubsystemFalcon1;
  public static WPI_TalonFX shooterSubsystemFalcon2;
  public static CANSparkMax shooterSubsystemHoodMax;
  public static RelativeEncoder shooterSubsystemHoodEncoder;
  public static DigitalInput hoodLimitSwitch;
  public static CANSparkMax shooterPreshooter;

  // turret
  public static CANSparkMax turretSubsystemturretSpinner;
  public static RelativeEncoder turretSubsystemturretEncoder;

  // climber
  public static DigitalInput climberStationaryHookContact;
  public static WPI_TalonFX climberExtentionMotor; 
  public static DoubleSolenoid climberArmTilt;

  // subsystems here...
  public static DriveSubsystem driveSubsystem;
  public static VisionSubsystem visionSubsystem;
  public static ClimberSubsystem climberSubsystem; 
  public static IntakeSubsystem intakeSubsystem;
  public static TurretSubsystem turretSubsystem;
  public static ShooterSubsystem shooterSubsystem;

  // joysticks here....
  public static Joystick driverJoystick;
  public static Joystick operatorJoystick;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();
    logger.info ("CAN bus: " + canDeviceFinder.getDeviceSet());

    makeHardware();
    setupMotors();
    makeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
    setupSmartDashboardCommands();
    setupAutonomousCommands();
  }

  void makeHardware() {
    practiceBotJumper = new DigitalInput(0);
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn ("this is a test chassis, will try to deal with missing hardware!");
    }

    // we don't *need* to use the canDeviceFinder for CAN Talons because
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

    climberStationaryHookContact = new DigitalInput(1);
    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON, 40, "climberExtentionMotor") || iAmACompetitionRobot) {
      climberExtentionMotor = new WPI_TalonFX(40);
    }

    // shooter motors
    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON, 21, "top shooter 1") || iAmACompetitionRobot) {
      // Shooter Motors 
      shooterSubsystemFalcon1 = new WPI_TalonFX(21);
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON, 17, "top shooter 2") || iAmACompetitionRobot) {
      shooterSubsystemFalcon2 = new WPI_TalonFX(17);
    }

    // turret 
    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 20, "turret") || iAmACompetitionRobot) {
      turretSubsystemturretSpinner = new CANSparkMax(20, MotorType.kBrushless);
      resetMaxToKnownState(turretSubsystemturretSpinner, true);
      turretSubsystemturretSpinner.setSmartCurrentLimit(10);
      turretSubsystemturretEncoder = turretSubsystemturretSpinner.getEncoder();
    }

    PneumaticsModuleType pneumaticModuleType = null;

    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || iAmACompetitionRobot) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }

    if (pneumaticModuleType == PneumaticsModuleType.CTREPCM) {
      Compressor compressor = new Compressor(pneumaticModuleType);
      compressor.disable();
    }

    if (pneumaticModuleType != null) {
      ringLight = new Solenoid(pneumaticModuleType, 7);
      ringLight.set(true);
      climberArmTilt = new DoubleSolenoid(pneumaticModuleType, 0, 1);
    }

    // intake
    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 25, "wheel bar") || iAmACompetitionRobot){
      intakeWheelbar = new CANSparkMax(25, MotorType.kBrushless);
    }
    /*
    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 26, "Intake Belt") || iAmACompetitionRobot){
      intakeBelt = new CANSparkMax(#, MotorType.UNDECIDED);
    }
      */
  }
  
  void setupMotors() {
    int kTimeoutMs = 0;

    if (driveSubsystemRightFrontDrive != null){

      resetMaxToKnownState(driveSubsystemRightFrontDrive, true);
      driveSubsystemRightFrontDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemRightFrontAzimuth, false);
      driveSubsystemRightFrontAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftFrontDrive, true);
      driveSubsystemLeftFrontDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftFrontAzimuth, false);
      driveSubsystemLeftFrontAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftBackDrive, true);
      driveSubsystemLeftBackDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftBackAzimuth, false);
      driveSubsystemLeftBackAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemRightBackDrive, true);
      driveSubsystemRightBackDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);
      
      resetMaxToKnownState(driveSubsystemRightBackAzimuth, false);
      driveSubsystemRightBackAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);
    }
  }

  static void resetMaxToKnownState(CANSparkMax x, boolean inverted) {
    x.setInverted(inverted);
    x.setIdleMode(IdleMode.kCoast);
    x.setOpenLoopRampRate(1);
    x.setClosedLoopRampRate(1);
    x.setSmartCurrentLimit(50);
  }

  void makeSubsystems() {
    driveSubsystem = new DriveSubsystem();
    visionSubsystem = new VisionSubsystem();
    climberSubsystem = new ClimberSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    turretSubsystem = new TurretSubsystem();
    shooterSubsystem = new ShooterSubsystem();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link edu.wpi.first.wpilibj.XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
    operatorJoystick = new Joystick(OPERATOR_JOYSTICK_PORT);

    //Dpad 
    DPad driverDPad = new DPad(driverJoystick, 0);
    DPad operatorDPad = new DPad(operatorJoystick, 0);

    //Climber Tilt Buttons
    JoystickButton climberTiltOutButton = new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A);
    climberTiltOutButton.whenPressed(new ClimberTiltTestCommandOut());
    JoystickButton climberTiltInButton = new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_B);
    climberTiltInButton.whenPressed(new ClimberTiltTestCommandIn());
    
    driverDPad.up().whenPressed(new MoveTurretCommand(turretSubsystem, 0));
    driverDPad.down().whenPressed(new MoveTurretCommand(turretSubsystem, 180));
    driverDPad.left().whenPressed(new MoveTurretCommand(turretSubsystem, 270));
    driverDPad.right().whenPressed(new MoveTurretCommand(turretSubsystem, 90));

    JoystickButton centerOnBallButton = new JoystickButton(driverJoystick, XBoxConstants.BUTTON_Y);
    centerOnBallButton.whileHeld(new InstantCenterOnBallCommand(driveSubsystem, visionSubsystem));

    AnalogJoystickButton climberExtendUp = new AnalogJoystickButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_Y);
    climberExtendUp.whileHeld(new ClimberTestCommandUp());
    AnalogJoystickButton climberExtendDown = new AnalogJoystickButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_Y);
    climberExtendDown.whileHeld(new ClimberTestCommandDown());
  }

  void setupSmartDashboardCommands() {
    SmartDashboard.putData(new ZeroDriveEncodersCommand(driveSubsystem));
  
    SmartDashboard.putData("TestAuto", new TestAuto(driveSubsystem));
    SmartDashboard.putData("AutoDriveToCargo Test", new DriveToCargoTestAuto(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("5 Ball Auto P", new FiveBallAuto(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("4 Ball Auto P", new FourBallAutoP(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("4 Ball Auto Q", new FourBallAutoQ(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("3 Ball Auto Q", new ThreeBallAutoQ(driveSubsystem,  visionSubsystem));

    SmartDashboard.putData("DougTestAutoDrive", new DougTestAutoDrive(driveSubsystem));
    SmartDashboard.putData("DougTestAutoSpin", new DougTestAutoSpin(driveSubsystem));
    SmartDashboard.putData("Reset NavX", new ResetNavXCommand(driveSubsystem));
    SmartDashboard.putData("Toggle field relative", new ToggleFieldRelativeModeCommand(driveSubsystem));
    SmartDashboard.putData("Find target",new FindTargetCommand(turretSubsystem, visionSubsystem));

    SmartDashboard.putData("Shooter Test Command", new ShooterTestCommand(shooterSubsystem));

    SmartDashboard.putData("Eject Ball", new EjectBallCommand());
    SmartDashboard.putData("Intake Ball", new IntakeBallCommand());

    SmartDashboard.putData("Climber Extention Motor Up", new ClimberTestCommandUp());
    SmartDashboard.putData("Climber Extention Motor Down", new ClimberTestCommandDown());
    SmartDashboard.putData("Climber Tilt Out", new ClimberTiltTestCommandOut());
    SmartDashboard.putData("Climber Tilt In", new ClimberTiltTestCommandIn());

    SmartDashboard.putData("Find target",new FindTargetCommand(turretSubsystem, visionSubsystem));
  }

  SendableChooser<Command> chooser = new SendableChooser<>();
  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);

    chooser.addOption("TestAuto", new TestAuto(driveSubsystem));
    chooser.addOption("AutoDriveToCargo Test", new DriveToCargoTestAuto(driveSubsystem, visionSubsystem));
    chooser.addOption("5 Ball P Auto", new FiveBallAuto(driveSubsystem, visionSubsystem));
    chooser.addOption("4 Ball P Auto", new FourBallAutoP(driveSubsystem, visionSubsystem));
    chooser.addOption("4 Ball Q Auto", new FourBallAutoQ(driveSubsystem, visionSubsystem));
    chooser.addOption("3 Ball Q Auto", new ThreeBallAutoQ(driveSubsystem, visionSubsystem));
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
    SmartDashboard.putNumber("driver.raw.spin", axisValue);
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

  public static double getOperatorVerticalJoystick() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_Y);
    if (axisValue < 0.15 && axisValue > -0.15) {
      return 0;
    }
    return -axisValue;
  }

  /**
   * Determine if this robot is a competition robot. It is if
   * it's connected to an FMS.
   * 
   * We should probably also check for an "I am a test" file or jumper
   * and return true if those are missing.
   * 
   * @return true if this robot is a competition robot.
   */

  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if(practiceBotJumper.get() == true){
      return true;
    }

    return false;
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new GoldenAutoCommand(driveSubsystem, shooterSubsystem, VisionSubsystem, intakeSubsystem);
    return chooser.getSelected();
  }
}